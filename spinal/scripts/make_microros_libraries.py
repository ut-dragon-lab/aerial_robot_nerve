#!/usr/bin/env python3
import argparse
import shutil
import subprocess
import re
from pathlib import Path
from typing import Optional


DEFAULT_DOCKER_IMAGE = "microros/micro_ros_static_library_builder:humble"

DEFAULT_MICROROS_FOLDER = "micro_ros_stm32cubemx_utils/microros_static_library_ide"
DEFAULT_MICROROS_EXTRA_DIR = (
    "micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/extra_packages"
)

DEFAULT_STAGE_SUBDIR = "microros"


def run(cmd: list[str], cwd: Optional[Path] = None) -> None:
    print(f"[cmd] {' '.join(cmd)}")
    subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=True)


def ensure_file(path: Path, initial_content: str = "") -> None:
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(initial_content, encoding="utf-8")


def remove_if_exists(path: Path) -> None:
    if path.is_symlink() or path.is_file():
        path.unlink()
    elif path.is_dir():
        shutil.rmtree(path)


def copy_tree(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(src, dst, dirs_exist_ok=True)


def validate_ros_interface_pkg(pkg_dir: Path) -> None:
    if not (pkg_dir / "package.xml").is_file():
        raise SystemExit(f"[error] package.xml not found in: {pkg_dir}")
    if not (pkg_dir / "CMakeLists.txt").is_file():
        raise SystemExit(f"[error] CMakeLists.txt not found in: {pkg_dir}")
    has_msg = (pkg_dir / "msg").is_dir()
    has_srv = (pkg_dir / "srv").is_dir()
    if not (has_msg or has_srv):
        raise SystemExit(f"[error] neither msg/ nor srv/ found in: {pkg_dir}")


def update_extra_repos(repos_yaml: Path, pkg_name: str) -> None:
    ensure_file(repos_yaml, "repositories:\n")
    text = repos_yaml.read_text(encoding="utf-8")

    entry = (
        f"\n  {pkg_name}:\n"
        f"    type: local\n"
        f"    path: extra_packages/{pkg_name}\n"
    )

    if f"\n  {pkg_name}:\n" in text:
        print(f"[info] extra_packages.repos already contains: {pkg_name}")
        return

    repos_yaml.write_text(text.rstrip() + entry, encoding="utf-8")
    print(f"[info] updated: {repos_yaml}")


def locate_artifacts(microros_root: Path) -> tuple[Path, Path]:
    libs = list(microros_root.rglob("libmicroros.a"))
    if not libs:
        raise SystemExit(f"[error] libmicroros.a not found under: {microros_root}")

    lib_path = libs[0]

    include_dir = lib_path.parent / "include"
    if include_dir.is_dir():
        return lib_path, include_dir

    include_candidates = [p for p in microros_root.rglob("include") if p.is_dir()]
    if not include_candidates:
        raise SystemExit(f"[error] include/ directory not found under: {microros_root}")

    return lib_path, include_candidates[0]


def infer_ws_src_from_script_location() -> Path:
    # Expected script location:
    #   <ws_src>/aerial_robot_nerve/spinal/scripts/<this_script>.py
    here = Path(__file__).resolve()
    try:
        ws_src = here.parents[3]
    except IndexError:
        raise SystemExit(f"[error] cannot infer ws/src from script location: {here}")
    return ws_src


def edit_stm32hardware_macros(src_h: Path, dst_h: Path, support_rtos: bool, support_lwip: bool) -> None:
    text = src_h.read_text(encoding="utf-8", errors="ignore")

    def set_macro(name: str, enabled: bool, content: str) -> str:
        value = "1" if enabled else "0"
        pattern = rf"(^\s*#\s*define\s+{re.escape(name)}\s+)[01](\s*(?:\/\/.*)?$)"
        if re.search(pattern, content, flags=re.MULTILINE):
            return re.sub(pattern, rf"\g<1>{value}\g<2>", content, flags=re.MULTILINE)
        return content

    text = set_macro("SUPPORT_RTOS", support_rtos, text)
    text = set_macro("SUPPORT_LWIP", support_lwip, text)

    dst_h.parent.mkdir(parents=True, exist_ok=True)
    dst_h.write_text(text, encoding="utf-8")


def stage_micro_ros_lib_files(
    spinal_pkg_dir: Path,
    stm32_microros_dir: Path,
    support_rtos: bool,
    support_lwip: bool
) -> None:
    src_dir = spinal_pkg_dir / "src" / "micro_ros_lib"
    if not src_dir.is_dir():
        raise SystemExit(f"[error] micro_ros_lib directory not found: {src_dir}")

    stm32_microros_dir.mkdir(parents=True, exist_ok=True)

    files = [
        "STM32Hardware.h",
        "STM32Hardware.cpp",
        "microros_transport_stm32.cpp",
        "microros_transport_stm32.h",
    ]

    for name in files:
        src = src_dir / name
        if not src.is_file():
            raise SystemExit(f"[error] required file not found: {src}")

    edit_stm32hardware_macros(
        src_h=src_dir / "STM32Hardware.h",
        dst_h=stm32_microros_dir / "STM32Hardware.h",
        support_rtos=support_rtos,
        support_lwip=support_lwip,
    )

    for name in ["STM32Hardware.cpp", "microros_transport_stm32.cpp", "microros_transport_stm32.h"]:
        shutil.copy2(src_dir / name, stm32_microros_dir / name)

    utils_src = src_dir / "utils"
    if utils_src.is_dir():
        utils_dst = stm32_microros_dir / "utils"
        copy_tree(utils_src, utils_dst)
        print(f"[done] staged utils directory: {utils_dst}")
    else:
        raise SystemExit(f"[error] utils directory not found: {utils_src}")

    print("[done] staged transport/hardware sources:")
    for name in files:
        print(f"  - {stm32_microros_dir / name}")


def main() -> None:
    inferred_ws_src = infer_ws_src_from_script_location()
    inferred_nerve_root = inferred_ws_src / "aerial_robot_nerve"

    ap = argparse.ArgumentParser(
        description=(
            "Inject spinal_msgs into micro-ROS extra_packages, build libmicroros.a with Docker, "
            "and stage artifacts into STM32 project."
        )
    )

    ap.add_argument(
        "--support_rtos",
        action="store_true",
        help="Enable RTOS support macro in STM32Hardware.h"
    )
    ap.add_argument(
        "--support_ethernet",
        action="store_true",
        help="Enable LWIP support macro in STM32Hardware.h"
    )
    ap.add_argument(
        "--ws-src",
        type=Path,
        default=inferred_ws_src,
        help="Workspace src directory to mount to /project (default inferred from script path).",
    )
    ap.add_argument(
        "--docker-image",
        type=str,
        default=DEFAULT_DOCKER_IMAGE,
        help="Docker image to use.",
    )
    ap.add_argument(
        "--microros-folder",
        type=str,
        default=DEFAULT_MICROROS_FOLDER,
        help="MICROROS_LIBRARY_FOLDER (relative to /project).",
    )
    ap.add_argument(
        "--microros-extra-dir",
        type=str,
        default=DEFAULT_MICROROS_EXTRA_DIR,
        help="extra_packages directory (relative to ws/src).",
    )
    ap.add_argument(
        "--spinal-msgs",
        type=Path,
        default=inferred_nerve_root / "spinal_msgs",
        help="Path to spinal_msgs package (default inferred under aerial_robot_nerve/spinal_msgs).",
    )
    ap.add_argument(
        "--stm32-lib",
        type=Path,
        default=inferred_nerve_root / "spinal/mcu_project/lib",
        help="STM32 project root directory (default inferred under aerial_robot_nerve/spinal/mcu_project/boards/stm32H7_v2).",
    )
    ap.add_argument(
        "--stage-subdir",
        type=str,
        default=DEFAULT_STAGE_SUBDIR,
        help="Destination subdir under STM32 project.",
    )
    ap.add_argument(
        "--clean-stage",
        action="store_true",
        help="Deprecated: staged directory is always cleaned before copying.",
    )
    ap.add_argument(
        "--clean-extra",
        action="store_true",
        help="Delete existing extra_packages/spinal_msgs before copying.",
    )

    args = ap.parse_args()

    ws_src: Path = args.ws_src.expanduser().resolve()
    if not ws_src.is_dir():
        raise SystemExit(f"[error] ws-src not found: {ws_src}")

    microros_root = ws_src / args.microros_folder
    if not microros_root.is_dir():
        raise SystemExit(f"[error] micro-ROS folder not found: {microros_root}")
    microros_build_output = microros_root / "libmicroros"

    spinal_msgs_src = args.spinal_msgs.expanduser().resolve()
    validate_ros_interface_pkg(spinal_msgs_src)

    stm32_lib: Path = args.stm32_lib.expanduser().resolve()
    if not stm32_lib.is_dir():
        raise SystemExit(f"[error] stm32-proj not found: {stm32_lib}")

    extra_dir = (ws_src / args.microros_extra_dir).resolve()
    repos_yaml = extra_dir / "extra_packages.repos"
    pkg_name = spinal_msgs_src.name
    spinal_msgs_dst = extra_dir / pkg_name

    extra_dir.mkdir(parents=True, exist_ok=True)

    if args.clean_extra:
        remove_if_exists(spinal_msgs_dst)

    if spinal_msgs_dst.exists():
        remove_if_exists(spinal_msgs_dst)

    copy_tree(spinal_msgs_src, spinal_msgs_dst)
    print(f"[info] synced extra package: {spinal_msgs_dst}")

    update_extra_repos(repos_yaml, pkg_name)

    docker_cmd_prefix = ["sudo", "docker"]

    if microros_build_output.exists():
        remove_if_exists(microros_build_output)
        print(f"[info] removed stale micro-ROS build output: {microros_build_output}")

    run(docker_cmd_prefix + ["pull", args.docker_image])

    run(
        docker_cmd_prefix
        + [
            "run",
            "--rm",
            "-v",
            f"{ws_src}:/project",
            "-e",
            f"MICROROS_LIBRARY_FOLDER={args.microros_folder}",
            args.docker_image,
        ]
    )

    lib_path, include_dir = locate_artifacts(microros_root)
    print(f"[info] libmicroros.a: {lib_path}")
    print(f"[info] include dir : {include_dir}")

    stage_root = stm32_lib / args.stage_subdir
    if stage_root.exists():
        remove_if_exists(stage_root)
        print(f"[info] removed stale staged micro-ROS files: {stage_root}")

    lib_dst_dir = stage_root / "lib"
    inc_dst_dir = stage_root / "include"

    lib_dst_dir.mkdir(parents=True, exist_ok=True)
    inc_dst_dir.mkdir(parents=True, exist_ok=True)

    shutil.copy2(lib_path, lib_dst_dir / "libmicroros.a")
    copy_tree(include_dir, inc_dst_dir)

    spinal_pkg_dir = (ws_src / "aerial_robot_nerve" / "spinal").resolve()
    stm32_microros_dir = (stm32_lib / "microros").resolve()

    stage_micro_ros_lib_files(
        spinal_pkg_dir=spinal_pkg_dir,
        stm32_microros_dir=stm32_microros_dir,
        support_rtos=args.support_rtos,
        support_lwip=args.support_ethernet,
    )

    print("[done] staged micro-ROS artifacts:")
    print(f"  - {lib_dst_dir / 'libmicroros.a'}")
    print(f"  - {inc_dst_dir}")
    print("[done] spinal_msgs injected via extra_packages:")
    print(f"  - {spinal_msgs_dst}")
    print(f"  - {repos_yaml}")

    print("CubeIDE typical settings:")
    print(f"  Include path: {args.stage_subdir}/include")
    print(f"  Library path: {args.stage_subdir}/lib")
    print("  Library     : microros")


if __name__ == "__main__":
    main()
