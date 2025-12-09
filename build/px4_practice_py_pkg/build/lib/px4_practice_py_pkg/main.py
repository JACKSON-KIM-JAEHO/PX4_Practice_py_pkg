import rclpy
import os
import sys
import threading
from ament_index_python.packages import get_package_share_directory
import py_trees
from py_trees.xml import loader as xml_loader
from py_trees.display import render_dot_tree
from py_trees.blackboard import Blackboard
from py_trees.common import Status

# BT 노드 및 ROS2NodeShared
from px4_practice_py_pkg.bt_nodes import (OffboardTakeoff, ROS2NodeShared)


def register_all_behaviours(shared_resources) -> dict:
    return {
        "OffboardTakeoff": lambda name, *args, **kwargs: OffboardTakeoff(name, shared_resources)
    }


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)

    shared_resources = ROS2NodeShared(node_name='bt_runner_node')
    node = shared_resources

    try:
        package_name = 'PX4_Practice_py_pkg'
        pkg_share_dir = get_package_share_directory(package_name)
    except Exception:
        node.get_logger().error("패키지 공유 디렉토리 찾기 실패.")
        shared_resources.executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return

    tree_file = os.path.join(pkg_share_dir, 'bt_xml', 'main_bt.xml')

    try:
        if not os.path.exists(tree_file):
            raise FileNotFoundError(f"XML 파일 경로를 찾을 수 없습니다: {tree_file}")

        behaviour_library = register_all_behaviours(shared_resources)

        # XML에서 Behavior Tree 불러오기
        root = xml_loader.import_from_xml(tree_file, behaviour_library)

        node.get_logger().info("✅ Behavior Tree 로드 완료. 실행 준비.")
        print(render_dot_tree(root))

        root.setup_with_verbose_printing()

        rate = node.create_rate(10)

        while rclpy.ok() and root.status == Status.RUNNING:
            status = root.tick()

            if status == Status.SUCCESS:
                node.get_logger().info("✅ 트리 성공적으로 종료됨.")
                break
            elif status == Status.FAILURE:
                node.get_logger().error("❌ 트리 실패 상태로 종료됨!")
                break

            rate.sleep()

    except FileNotFoundError as e:
        node.get_logger().error(f"❌ 파일 로드 실패: {e}")
    except Exception as e:
        node.get_logger().error(f"BT 실행 중 치명적인 오류 발생: {e}",
                                throttle_duration_sec=1.0)

    finally:
        node.get_logger().info("🛑 시스템 종료 중...")
        shared_resources.executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
