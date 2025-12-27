# px4_practice_py_pkg/modules/bt_constructor.py

import os

from px4_practice_py_pkg.modules.utils import (
    parse_behavior_tree,
    convert_value,
    get_file_dirname,
)

# 🔥 BT 노드 정의는 fire 시나리오에 고정
from px4_practice_py_pkg.scenario.fire.bt_nodes import bt_nodes


def build_behavior_tree(shared_node, behavior_tree_xml: str):
    """
    Build a Behavior Tree from XML.
    This BT is used as a mission orchestrator (NOT agent-based).

    Parameters
    ----------
    shared_node : rclpy.node.Node
        Shared ROS2 node (MAVROS interface, state, publishers, subscribers).
    behavior_tree_xml : str
        Path to BT XML file.

    Returns
    -------
    root_node : BT Node
    """

    xml_root = parse_behavior_tree(behavior_tree_xml)

    bt_root = xml_root.find("BehaviorTree")
    if bt_root is None:
        raise ValueError("[BT ERROR] <BehaviorTree> tag not found in XML.")

    return _parse_xml_node(
        bt_root,
        shared_node=shared_node,
        top_xml_path=behavior_tree_xml,
    )


def _parse_xml_node(xml_node, *, shared_node, top_xml_path):
    node_type = xml_node.tag

    # -------------------------------------------------
    # SubTree include (ID.xml)
    # -------------------------------------------------
    if node_type == "SubTree":
        subtree_id = xml_node.attrib.get("ID")
        if not subtree_id:
            raise ValueError("[BT ERROR] <SubTree> must have an ID attribute.")

        base_dir = get_file_dirname(top_xml_path)
        subtree_xml_path = os.path.join(base_dir, f"{subtree_id}.xml")

        subtree_root = parse_behavior_tree(subtree_xml_path).find("BehaviorTree")
        if subtree_root is None:
            raise ValueError(f"[BT ERROR] SubTree file '{subtree_id}.xml' has no <BehaviorTree>.")

        return _parse_xml_node(
            subtree_root,
            shared_node=shared_node,
            top_xml_path=subtree_xml_path,
        )

    # -------------------------------------------------
    # Parse children first (DFS)
    # -------------------------------------------------
    children = [
        _parse_xml_node(
            child,
            shared_node=shared_node,
            top_xml_path=top_xml_path,
        )
        for child in xml_node
    ]

    attrib = {k: convert_value(v) for k, v in xml_node.attrib.items()}
    BTNodeList = bt_nodes.BTNodeList

    # -------------------------------------------------
    # Control Flow Nodes
    # -------------------------------------------------
    if node_type in BTNodeList.CONTROL_NODES:
        node_class = getattr(bt_nodes, node_type)
        return node_class(
            name=node_type,
            children=children,
            **attrib,
        )

    # -------------------------------------------------
    # Decorator Nodes
    # -------------------------------------------------
    elif node_type in BTNodeList.DECORATOR_NODES:
        if len(children) != 1:
            raise ValueError(
                f"[BT ERROR] Decorator '{node_type}' must have exactly one child."
            )

        node_class = getattr(bt_nodes, node_type)
        return node_class(
            name=node_type,
            child=children[0],
            **attrib,
        )

    # -------------------------------------------------
    # Action / Condition Nodes
    # -------------------------------------------------
    elif node_type in (BTNodeList.ACTION_NODES + BTNodeList.CONDITION_NODES):
        node_class = getattr(bt_nodes, node_type)
        return node_class(
            name=node_type,
            shared=shared_node,
            **attrib,
        )

    # -------------------------------------------------
    # Root passthrough
    # -------------------------------------------------
    elif node_type == "BehaviorTree":
        if not children:
            raise ValueError("[BT ERROR] <BehaviorTree> has no child.")
        return children[0]

    else:
        raise ValueError(f"[BT ERROR] Unknown BT node type: {node_type}")
