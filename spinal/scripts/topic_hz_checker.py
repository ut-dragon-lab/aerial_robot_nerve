#!/usr/bin/env python3
import argparse
from collections import deque
import statistics
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from rosidl_runtime_py.utilities import get_message


HISTORY_POLICIES = {
    'keep_last': HistoryPolicy.KEEP_LAST,
    'keep_all': HistoryPolicy.KEEP_ALL,
    'system_default': HistoryPolicy.SYSTEM_DEFAULT,
}

RELIABILITY_POLICIES = {
    'best_effort': ReliabilityPolicy.BEST_EFFORT,
    'reliable': ReliabilityPolicy.RELIABLE,
    'system_default': ReliabilityPolicy.SYSTEM_DEFAULT,
}

DURABILITY_POLICIES = {
    'volatile': DurabilityPolicy.VOLATILE,
    'transient_local': DurabilityPolicy.TRANSIENT_LOCAL,
    'system_default': DurabilityPolicy.SYSTEM_DEFAULT,
}


def normalize_type_name(type_name):
    parts = type_name.split('/')
    if len(parts) == 2:
        return f'{parts[0]}/msg/{parts[1]}'
    return type_name


def parse_args():
    parser = argparse.ArgumentParser(
        description='Measure a ROS 2 topic receive rate with configurable QoS.'
    )
    parser.add_argument(
        'topic',
        nargs='?',
        default='/imu',
        help='Topic name to subscribe to. Defaults to /imu.',
    )
    parser.add_argument(
        '-t',
        '--type',
        dest='type_name',
        help='Message type, for example spinal_msgs/msg/Imu. Auto-detected if omitted.',
    )
    parser.add_argument(
        '--wait-timeout',
        type=float,
        default=5.0,
        help='Seconds to wait for auto type detection. Use 0 to wait forever.',
    )
    parser.add_argument(
        '--reliability',
        choices=sorted(RELIABILITY_POLICIES),
        default='best_effort',
        help='Subscriber QoS reliability.',
    )
    parser.add_argument(
        '--history',
        choices=sorted(HISTORY_POLICIES),
        default='keep_last',
        help='Subscriber QoS history policy.',
    )
    parser.add_argument(
        '--depth',
        type=int,
        default=100,
        help='Subscriber QoS depth for keep_last history.',
    )
    parser.add_argument(
        '--durability',
        choices=sorted(DURABILITY_POLICIES),
        default='volatile',
        help='Subscriber QoS durability.',
    )
    parser.add_argument(
        '--window',
        type=int,
        default=100,
        help='Number of recent messages used for rolling statistics.',
    )
    parser.add_argument(
        '--print-period',
        type=float,
        default=1.0,
        help='Seconds between rate printouts.',
    )
    parser.add_argument(
        '--spin-timeout',
        type=float,
        default=0.01,
        help='Seconds passed to rclpy.spin_once().',
    )
    return parser.parse_args(remove_ros_args()[1:])


def find_topic_types(node, topic_name):
    for name, type_names in node.get_topic_names_and_types():
        if name == topic_name:
            return type_names
    return []


def detect_type_name(node, topic_name, wait_timeout):
    start = time.monotonic()
    while rclpy.ok():
        type_names = find_topic_types(node, topic_name)
        if len(type_names) == 1:
            return type_names[0]
        if len(type_names) > 1:
            raise RuntimeError(
                f'topic {topic_name} has multiple types: {", ".join(type_names)}; '
                'please specify --type'
            )
        if wait_timeout > 0.0 and time.monotonic() - start >= wait_timeout:
            raise RuntimeError(
                f'could not detect type for {topic_name} within {wait_timeout:.1f} s; '
                'please specify --type'
            )
        rclpy.spin_once(node, timeout_sec=0.1)


def make_qos(args):
    if args.depth <= 0:
        raise ValueError('--depth must be greater than 0')
    return QoSProfile(
        history=HISTORY_POLICIES[args.history],
        depth=args.depth,
        reliability=RELIABILITY_POLICIES[args.reliability],
        durability=DURABILITY_POLICIES[args.durability],
    )


def format_window_stats(stamps):
    if len(stamps) < 2:
        return 'window: collecting'

    periods = [b - a for a, b in zip(stamps, list(stamps)[1:])]
    elapsed = stamps[-1] - stamps[0]
    if elapsed <= 0.0:
        return 'window: invalid'

    rate = (len(stamps) - 1) / elapsed
    mean_period = statistics.mean(periods)
    min_period = min(periods)
    max_period = max(periods)
    std_period = statistics.pstdev(periods) if len(periods) > 1 else 0.0
    return (
        f'window: {rate:.2f} Hz '
        f'mean: {mean_period * 1000.0:.3f} ms '
        f'min: {min_period * 1000.0:.3f} ms '
        f'max: {max_period * 1000.0:.3f} ms '
        f'std: {std_period * 1000.0:.3f} ms'
    )


def main():
    args = parse_args()

    rclpy.init()
    node = rclpy.create_node('topic_hz_checker')

    try:
        type_name = args.type_name
        if type_name is None:
            type_name = detect_type_name(node, args.topic, args.wait_timeout)

        msg_type = get_message(normalize_type_name(type_name))
        qos = make_qos(args)

        count = 0
        interval_count = 0
        last_print = time.monotonic()
        stamps = deque(maxlen=max(args.window, 2))

        def cb(_msg):
            nonlocal count, interval_count
            now = time.monotonic()
            count += 1
            interval_count += 1
            stamps.append(now)

        node.create_subscription(msg_type, args.topic, cb, qos)
        print(
            f'subscribed: topic={args.topic} type={normalize_type_name(type_name)} '
            f'reliability={args.reliability} history={args.history} '
            f'depth={args.depth} durability={args.durability}'
        )

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=args.spin_timeout)
            now = time.monotonic()
            elapsed = now - last_print
            if elapsed >= args.print_period:
                interval_rate = interval_count / elapsed if elapsed > 0.0 else 0.0
                print(
                    f'average: {interval_rate:.2f} Hz '
                    f'count: {count} {format_window_stats(stamps)}'
                )
                interval_count = 0
                last_print = now
    except KeyboardInterrupt:
        return 0
    except (AttributeError, ModuleNotFoundError, RuntimeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
