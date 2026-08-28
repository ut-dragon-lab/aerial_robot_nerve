#!/usr/bin/env python3

"""Quiesce spinal through ROS and flash it through UART or ST-LINK/SWD."""

import argparse
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from typing import List, NamedTuple, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


APPLICATION_ADDRESS = "0x08000000"
CM7_BOOT_ADDRESS_REGISTER = "0x58000708"
MIN_CUBEPROGRAMMER_VERSION = (2, 23, 0)


class FirmwareUpdateError(RuntimeError):
    pass


class ProcessRecord(NamedTuple):
    pid: int
    argv: List[str]
    cwd: str


class Programmer(NamedTuple):
    executable: str
    backend: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Stop motor outputs, enter the STM32H743 ROM bootloader, and flash "
            "firmware through ST-LINK/SWD or microUSB/FT232/USART1."
        )
    )
    parser.add_argument(
        "--firmware",
        type=Path,
        help="Firmware .bin, .hex, or .elf (default: installed spinal.bin)",
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help=(
            "FT232 device used by the micro-ROS agent and UART programmer; "
            "/dev/serial/by-id/... is recommended"
        ),
    )
    parser.add_argument(
        "--interface",
        choices=("swd", "uart"),
        default="uart",
        help="programming interface (default: uart for backward compatibility)",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--service", default="/enter_bootloader")
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument(
        "--application-timeout",
        type=float,
        default=15.0,
        help="time to wait for the STM32 ROS service after programming",
    )
    parser.add_argument("--reset-wait", type=float, default=2.0)
    parser.add_argument(
        "--bootloader-timeout",
        type=float,
        default=30.0,
        help=(
            "maximum time to retry a non-destructive UART bootloader probe "
            "before programming (default: 30 seconds)"
        ),
    )
    parser.add_argument(
        "--programmer",
        help="Programmer executable override (normally auto-detected)",
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "cubeprogrammer"),
        default="auto",
        help="Programming backend (default: STM32CubeProgrammer)",
    )
    parser.add_argument(
        "--skip-bootloader-request",
        action="store_true",
        help="Flash a board that is already in the ROM bootloader",
    )
    parser.add_argument(
        "--agent-stop-command",
        help='Command used to release the serial port, e.g. "systemctl stop micro-ros-agent"',
    )
    parser.add_argument(
        "--agent-start-command",
        help='Command used after a successful flash, e.g. "systemctl start micro-ros-agent"',
    )
    parser.add_argument(
        "--no-restart-agent",
        action="store_true",
        help="Do not restart a stopped micro-ROS agent after flashing",
    )
    return parser.parse_args()


def default_firmware_path() -> Path:
    share = Path(get_package_share_directory("spinal_firmware"))
    return share / "firmware" / "spinal.bin"


def find_executable(name: str) -> Optional[str]:
    if os.sep in name:
        executable = Path(name).expanduser().resolve()
        if executable.is_file() and os.access(executable, os.X_OK):
            return str(executable)
    else:
        executable = shutil.which(name)
        if executable:
            return executable
    return None


def resolve_executable(name: str) -> str:
    executable = find_executable(name)
    if executable is not None:
        return executable
    raise FirmwareUpdateError(f"programmer executable not found: {name}")


def infer_backend(executable: str) -> str:
    name = Path(executable).name.lower()
    if "stm32_programmer_cli" in name:
        return "cubeprogrammer"
    raise FirmwareUpdateError(
        "cannot infer the programmer backend from the executable name; "
        "specify --backend cubeprogrammer"
    )


def resolve_programmer(backend: str, override: Optional[str]) -> Programmer:
    if override:
        executable = resolve_executable(override)
        inferred_backend = infer_backend(executable)
        if backend != "auto" and backend != inferred_backend:
            raise FirmwareUpdateError(
                f"programmer {executable} is {inferred_backend}, not {backend}"
            )
        selected_backend = inferred_backend if backend == "auto" else backend
        return Programmer(executable=executable, backend=selected_backend)

    data_root = Path(
        os.environ.get("XDG_DATA_HOME", str(Path.home() / ".local" / "share"))
    )
    candidates = [
        ("STM32_Programmer_CLI", "cubeprogrammer"),
        (
            str(
                data_root
                / "stm32cube"
                / "bundles"
                / "programmer"
                / "2.23.0"
                / "bin"
                / "STM32_Programmer_CLI"
            ),
            "cubeprogrammer",
        ),
    ]

    if backend != "auto":
        candidates = [
            candidate for candidate in candidates if candidate[1] == backend
        ]

    for name, candidate_backend in candidates:
        executable = find_executable(name)
        if executable is not None:
            return Programmer(executable=executable, backend=candidate_backend)

    names = ", ".join(name for name, _ in candidates)
    raise FirmwareUpdateError(
        f"no STM32 programmer found ({names}); install STM32CubeProgrammer "
        "2.23 or newer and put STM32_Programmer_CLI on PATH"
    )


def cubeprogrammer_version(executable: str) -> Tuple[int, int, int]:
    try:
        result = subprocess.run(
            [executable, "--version"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=10.0,
        )
    except subprocess.TimeoutExpired as exc:
        raise FirmwareUpdateError("STM32CubeProgrammer version check timed out") from exc

    match = re.search(
        r"STM32CubeProgrammer(?:\s+version:|\s+v)\s*"
        r"(\d+)\.(\d+)(?:\.(\d+))?",
        result.stdout,
        flags=re.IGNORECASE,
    )
    if result.returncode != 0 or match is None:
        raise FirmwareUpdateError(
            "could not determine STM32CubeProgrammer version from --version"
        )
    return tuple(int(part or 0) for part in match.groups())


def validate_programmer(programmer: Programmer) -> None:
    version = cubeprogrammer_version(programmer.executable)
    version_text = ".".join(str(part) for part in version)
    minimum_text = ".".join(str(part) for part in MIN_CUBEPROGRAMMER_VERSION)
    print(f"[flash] STM32CubeProgrammer version: {version_text}")
    if version < MIN_CUBEPROGRAMMER_VERSION:
        raise FirmwareUpdateError(
            f"STM32CubeProgrammer {version_text} is older than the validated "
            f"programming stack; install {minimum_text} or newer"
        )


def process_uses_port(pid: int, port_stat: os.stat_result) -> bool:
    fd_dir = Path("/proc") / str(pid) / "fd"
    try:
        descriptors = list(fd_dir.iterdir())
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return False

    for descriptor in descriptors:
        try:
            descriptor_stat = descriptor.stat()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if (
            descriptor_stat.st_dev == port_stat.st_dev
            and descriptor_stat.st_ino == port_stat.st_ino
            and descriptor_stat.st_rdev == port_stat.st_rdev
        ):
            return True
    return False


def read_process(pid: int) -> Optional[ProcessRecord]:
    proc_dir = Path("/proc") / str(pid)
    try:
        argv = [
            item.decode(errors="replace")
            for item in (proc_dir / "cmdline").read_bytes().split(b"\0")
            if item
        ]
        cwd = os.readlink(proc_dir / "cwd")
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return None
    if not argv:
        return None
    return ProcessRecord(pid=pid, argv=argv, cwd=cwd)


def port_users(port: Path) -> List[ProcessRecord]:
    try:
        port_stat = port.stat()
    except FileNotFoundError as exc:
        raise FirmwareUpdateError(f"serial port does not exist: {port}") from exc

    users: List[ProcessRecord] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == os.getpid() or not process_uses_port(pid, port_stat):
            continue
        record = read_process(pid)
        if record is not None:
            users.append(record)
    return users


def is_micro_ros_agent(process: ProcessRecord) -> bool:
    return any("micro_ros_agent" in argument for argument in process.argv)


def find_auto_managed_agent(port: Path) -> Optional[ProcessRecord]:
    users = port_users(port)
    if not users:
        return None
    if len(users) != 1 or not is_micro_ros_agent(users[0]):
        summary = ", ".join(
            f"pid={process.pid} ({Path(process.argv[0]).name})" for process in users
        )
        raise FirmwareUpdateError(
            f"serial port is occupied by a process that cannot be managed safely: {summary}"
        )
    try:
        os.kill(users[0].pid, 0)
    except PermissionError as exc:
        raise FirmwareUpdateError(
            "micro-ROS agent is owned by another user; use explicit privileged "
            "--agent-stop-command and --agent-start-command options"
        ) from exc
    except ProcessLookupError:
        return None
    return users[0]


def validate_words(command: str, description: str) -> List[str]:
    words = shlex.split(command)
    if not words:
        raise FirmwareUpdateError(f"empty {description} command")

    executable = words[0]
    if os.sep in executable:
        executable_path = Path(executable).expanduser()
        valid = executable_path.is_file() and os.access(executable_path, os.X_OK)
    else:
        valid = shutil.which(executable) is not None
    if not valid:
        raise FirmwareUpdateError(
            f"{description} command executable not found: {executable}"
        )
    return words


def run_words(command: str, description: str) -> None:
    words = validate_words(command, description)
    print(f"[flash] {description}: {shlex.join(words)}")
    result = subprocess.run(words, check=False)
    if result.returncode != 0:
        raise FirmwareUpdateError(
            f"{description} command failed with exit code {result.returncode}"
        )


def wait_for_process_exit(pid: int, timeout: float) -> bool:
    def process_is_running() -> bool:
        try:
            # A container PID 1 may not reap a terminated child immediately.
            # A zombie no longer owns the serial port and is safe to treat as
            # stopped even though /proc/<pid> still exists.
            fields = (Path("/proc") / str(pid) / "stat").read_text().split()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            return False
        return len(fields) > 2 and fields[2] != "Z"

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not process_is_running():
            return True
        time.sleep(0.1)
    return not process_is_running()


def stop_auto_managed_agent(agent: ProcessRecord) -> None:
    print(f"[flash] stopping micro-ROS agent pid={agent.pid}")
    try:
        # A background micro_ros_agent commonly inherits SIGINT as ignored from
        # its non-interactive shell.  Waiting for SIGINT here also lets it keep
        # transmitting while the MCU resets, which can corrupt the ROM
        # bootloader's UART auto-baud handshake.  SIGTERM releases the port
        # before the firmware's delayed reset.
        os.kill(agent.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    if wait_for_process_exit(agent.pid, 1.0):
        return

    print(f"[flash] agent did not stop on SIGTERM; sending SIGKILL to pid={agent.pid}")
    try:
        os.kill(agent.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    if not wait_for_process_exit(agent.pid, 0.5):
        raise FirmwareUpdateError(f"micro-ROS agent pid={agent.pid} did not stop")


def ensure_port_released(port: Path, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    users: List[ProcessRecord] = []
    while time.monotonic() < deadline:
        users = port_users(port)
        if not users:
            return
        time.sleep(0.1)
    summary = ", ".join(f"pid={process.pid}" for process in users)
    raise FirmwareUpdateError(f"serial port was not released ({summary})")


def restart_auto_managed_agent(agent: ProcessRecord) -> None:
    print(f"[flash] restarting micro-ROS agent: {shlex.join(agent.argv)}")
    subprocess.Popen(
        agent.argv,
        cwd=agent.cwd,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def request_bootloader(service_name: str, timeout: float) -> None:
    rclpy.init(args=None)
    node = Node("spinal_firmware_updater")
    try:
        client = node.create_client(Trigger, service_name)
        print(f"[flash] waiting for service {service_name}")
        if not client.wait_for_service(timeout_sec=timeout):
            raise FirmwareUpdateError(f"service not available: {service_name}")

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise FirmwareUpdateError("bootloader service call timed out")
        if future.exception() is not None:
            raise FirmwareUpdateError(f"bootloader service failed: {future.exception()}")

        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else "no response"
            raise FirmwareUpdateError(f"bootloader request rejected: {message}")
        print(f"[flash] STM32 acknowledged: {response.message}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def wait_for_application(service_name: str, timeout: float) -> None:
    rclpy.init(args=None)
    node = Node("spinal_firmware_post_flash_check")
    try:
        client = node.create_client(Trigger, service_name)
        print(f"[flash] waiting for application service {service_name}")
        if not client.wait_for_service(timeout_sec=timeout):
            raise FirmwareUpdateError(
                "firmware was written and verified, but the STM32H743 ROM "
                "bootloader did not return to the application; power-cycle "
                "the board (or use ST-LINK/SWD for a fully automatic reset)"
            )
        print("[flash] STM32 application is online")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def append_firmware_write(command: List[str], firmware: Path) -> None:
    command.extend(["-w", str(firmware)])
    if firmware.suffix.lower() == ".bin":
        command.append(APPLICATION_ADDRESS)
    # CubeProgrammer's fast sector verification is not supported by its UART
    # bootloader backend, so keep the legacy byte-by-byte verification here.
    command.extend(["-v", "-g", APPLICATION_ADDRESS])


def programmer_commands(
    programmer: Programmer,
    interface: str,
    port: Path,
    baud: int,
    firmware: Path,
) -> List[List[str]]:
    if interface == "uart":
        command = [
            programmer.executable,
            "--quietMode",
            "-c",
            f"port={port}",
            f"br={baud}",
        ]
        append_firmware_write(command, firmware)
        return [command]

    # /enter_bootloader changes the volatile CM7 boot-address shadow before
    # resetting. Restore it first so every reset after an SWD update returns to
    # the application even if programming is interrupted later.
    restore_boot_address = [
        programmer.executable,
        "--quietMode",
        "-c",
        "port=SWD",
        "mode=HOTPLUG",
        "-w32",
        CM7_BOOT_ADDRESS_REGISTER,
        APPLICATION_ADDRESS,
    ]
    program = [
        programmer.executable,
        "--quietMode",
        "-c",
        "port=SWD",
        "mode=HOTPLUG",
    ]
    append_firmware_write(program, firmware)
    return [restore_boot_address, program]


def wait_for_uart_bootloader(
    programmer: Programmer,
    port: Path,
    baud: int,
    timeout: float,
) -> None:
    """Wait until GET ID succeeds before allowing an erase/write command."""
    command = [
        programmer.executable,
        "--quietMode",
        "-c",
        f"port={port}",
        f"br={baud}",
    ]
    deadline = time.monotonic() + timeout
    attempt = 0
    last_output = ""

    while True:
        attempt += 1
        print(f"[flash] probing UART ROM bootloader (attempt {attempt})")
        result = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        last_output = result.stdout
        if result.returncode == 0 and re.search(
            r"Chip ID:\s*0x[1-9a-f][0-9a-f]*", last_output, re.IGNORECASE
        ):
            print("[flash] UART ROM bootloader is ready (GET ID succeeded)")
            return

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(2.0, remaining))

    detail = ""
    if "GETID command not acknowledged" in last_output:
        detail = ": GET ID was not acknowledged"
    raise FirmwareUpdateError(
        f"UART ROM bootloader did not become ready within {timeout:.1f}s{detail}"
    )


def flash(commands: List[List[str]], backend: str, interface: str) -> None:
    for command in commands:
        print(f"[flash] programming: {shlex.join(command)}")
        result = subprocess.run(command, check=False)
        if result.returncode != 0:
            if interface == "uart":
                recovery = (
                    "the board may still be in the ROM bootloader; restore it "
                    "with ST-LINK before retrying"
                )
            else:
                recovery = "check the ST-LINK connection before retrying"
            raise FirmwareUpdateError(f"{backend} failed; {recovery}")


def main() -> int:
    args = parse_args()
    try:
        firmware = (args.firmware or default_firmware_path()).expanduser().resolve()
        if not firmware.is_file():
            raise FirmwareUpdateError(f"firmware does not exist: {firmware}")
        if firmware.suffix.lower() not in {".bin", ".hex", ".elf"}:
            raise FirmwareUpdateError("firmware must have a .bin, .hex, or .elf suffix")
        if args.baud <= 0:
            raise FirmwareUpdateError("baud must be positive")

        port = Path(args.port).expanduser().resolve()
        programmer = resolve_programmer(args.backend, args.programmer)
        print(
            f"[flash] programmer backend: {programmer.backend} "
            f"({programmer.executable})"
        )
        # Validate before requesting a reset, since a preflight failure must not
        # leave the flight controller in system memory.
        validate_programmer(programmer)
        if args.agent_start_command and not args.agent_stop_command:
            raise FirmwareUpdateError(
                "--agent-start-command requires --agent-stop-command"
            )
        if (
            args.agent_stop_command
            and not args.agent_start_command
            and not args.no_restart_agent
        ):
            raise FirmwareUpdateError(
                "provide --agent-start-command or use --no-restart-agent"
            )
        if args.agent_stop_command:
            validate_words(args.agent_stop_command, "agent stop")
        if args.agent_start_command:
            validate_words(args.agent_start_command, "agent start")

        managed_agent = None
        if not args.agent_stop_command:
            managed_agent = find_auto_managed_agent(port)

        if not args.skip_bootloader_request:
            request_bootloader(args.service, args.service_timeout)

        if args.agent_stop_command:
            run_words(args.agent_stop_command, "agent stop")
        elif managed_agent is not None:
            stop_auto_managed_agent(managed_agent)

        ensure_port_released(port)
        print(f"[flash] waiting {args.reset_wait:.1f}s for the ROM bootloader")
        time.sleep(max(0.0, args.reset_wait))

        if args.interface == "uart":
            wait_for_uart_bootloader(
                programmer,
                port,
                args.baud,
                max(0.0, args.bootloader_timeout),
            )

        flash(
            programmer_commands(
                programmer,
                args.interface,
                port,
                args.baud,
                firmware,
            ),
            programmer.backend,
            args.interface,
        )

        if not args.no_restart_agent:
            if args.agent_start_command:
                run_words(args.agent_start_command, "agent start")
                wait_for_application(
                    args.service, max(0.0, args.application_timeout)
                )
            elif managed_agent is not None:
                restart_auto_managed_agent(managed_agent)
                wait_for_application(
                    args.service, max(0.0, args.application_timeout)
                )

        print("[flash] firmware update completed")
        return 0
    except (FirmwareUpdateError, OSError) as exc:
        print(f"[flash] error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
