#!/usr/bin/env python3
import argparse
import signal
import subprocess
import time


def start(cmd):
    return subprocess.Popen(cmd)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Start teleop_twist_joy with Ignition bridge and correct remaps."
    )
    parser.add_argument(
        "--cmd-topic",
        default="/model/limo_01/cmd_vel",
        help="Ignition cmd_vel topic.",
    )
    parser.add_argument(
        "--odom-topic",
        default="/model/limo_01/odometry",
        help="Ignition odometry topic.",
    )
    parser.add_argument("--enable-button", type=int, default=7, help="Enable button index.")
    parser.add_argument(
        "--require-enable",
        action="store_true",
        help="Require enable button to publish cmd_vel.",
    )
    parser.add_argument("--axis-linear", type=int, default=1, help="Linear axis index.")
    parser.add_argument("--axis-angular", type=int, default=3, help="Angular axis index.")
    parser.add_argument("--scale-linear", type=float, default=0.8, help="Linear scale.")
    parser.add_argument("--scale-angular", type=float, default=2.0, help="Angular scale.")
    parser.add_argument("--joy-deadzone", type=float, default=0.02, help="Joystick deadzone.")
    parser.add_argument("--autorepeat", type=float, default=50.0, help="joy_node autorepeat rate.")
    parser.add_argument("--coalesce", type=float, default=0.01, help="joy_node coalesce interval.")
    args = parser.parse_args()

    processes = []
    try:
        processes.append(
            start(
                [
                    "ros2",
                    "run",
                    "ros_ign_bridge",
                    "parameter_bridge",
                    f"{args.cmd_topic}@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                    f"{args.odom_topic}@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                ]
            )
        )

        processes.append(
            start(
                [
                    "ros2",
                    "run",
                    "joy",
                    "joy_node",
                    "--ros-args",
                    "-p",
                    f"autorepeat_rate:={args.autorepeat}",
                    "-p",
                    f"coalesce_interval:={args.coalesce}",
                    "-p",
                    f"deadzone:={args.joy_deadzone}",
                ]
            )
        )

        processes.append(
            start(
                [
                    "ros2",
                    "run",
                    "teleop_twist_joy",
                    "teleop_node",
                    "--ros-args",
                    "-r",
                    f"/cmd_vel:={args.cmd_topic}",
                    "-p",
                    f"enable_button:={args.enable_button}",
                    "-p",
                    f"require_enable_button:={str(args.require_enable).lower()}",
                    "-p",
                    f"axis_linear.x:={args.axis_linear}",
                    "-p",
                    f"axis_angular.yaw:={args.axis_angular}",
                    "-p",
                    f"scale_linear.x:={args.scale_linear}",
                    "-p",
                    f"scale_angular.yaw:={args.scale_angular}",
                ]
            )
        )

        while True:
            time.sleep(0.5)
            for proc in processes:
                if proc.poll() is not None:
                    return proc.returncode or 0
    except KeyboardInterrupt:
        pass
    finally:
        for proc in processes:
            if proc.poll() is None:
                try:
                    proc.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
        time.sleep(0.5)
        for proc in processes:
            if proc.poll() is None:
                try:
                    proc.terminate()
                except ProcessLookupError:
                    pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
