import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import os
import time
from ament_index_python.packages import get_package_share_directory

from .bt_nodes.shared_node import ROS2NodeShared
from px4_practice_py_pkg.modules.bt_constructor import build_behavior_tree
from px4_practice_py_pkg.modules.base_bt_nodes import Status


def main(args=None):
    rclpy.init(args=args)

    # 1️⃣ Shared ROS2 Node
    shared = ROS2NodeShared(node_name='bt_runner_node')

    # 2️⃣ Executor (ROS 통신 전용)
    executor = MultiThreadedExecutor()
    executor.add_node(shared)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 3️⃣ MAVROS 준비 대기
    shared.get_logger().info("[MAIN] Waiting for MAVROS data...")

    while rclpy.ok():
        if (shared.current_state.connected and
            shared.local_pose_received and
            shared.altitude_received):
            break
        time.sleep(0.05)

    shared.get_logger().info("[MAIN] MAVROS ready. Building BT...")

    # 4️⃣ BT XML 경로
    pkg_share = get_package_share_directory("px4_practice_py_pkg")
    xml_path = os.path.join(pkg_share, "scenario", "fire", "main_bt.xml")

    # 5️⃣ BT 생성
    root = build_behavior_tree(shared, xml_path)
    shared.get_logger().info("[MAIN] Behavior Tree started")

    # 6️⃣ BT Tick Loop (20Hz)
    try:
        while rclpy.ok():
            status = root.tick()

            if status == Status.SUCCESS:
                shared.get_logger().info("[BT] Mission completed")
                break

            if status == Status.FAILURE:
                shared.get_logger().warn("[BT] Mission failed")
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        shared.get_logger().warn("[MAIN] KeyboardInterrupt")

    finally:
        executor.shutdown()
        shared.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
