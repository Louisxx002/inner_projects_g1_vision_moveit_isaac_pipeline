from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive


def make_box(object_id: str, frame_id: str, size_xyz: tuple[float, float, float], center_xyz: tuple[float, float, float]) -> CollisionObject:
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size_xyz)

    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.x = float(center_xyz[0])
    pose.position.y = float(center_xyz[1])
    pose.position.z = float(center_xyz[2])

    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = object_id
    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return obj


class StaticScenePublisher(Node):
    def __init__(self) -> None:
        super().__init__("g1_static_scene_publisher")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(PlanningScene, "/planning_scene", qos)
        self.timer = self.create_timer(0.5, self.publish_once)
        self.done = False

    def publish_once(self) -> None:
        if self.done:
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [
            make_box("table_keepout", "pelvis", (0.90, 1.00, 0.04), (0.42, 0.0, -0.08)),
        ]
        remove_torso_keepout = CollisionObject()
        remove_torso_keepout.header.frame_id = "pelvis"
        remove_torso_keepout.id = "torso_front_keepout"
        remove_torso_keepout.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(remove_torso_keepout)
        self.pub.publish(scene)
        self.get_logger().info("Published static planning scene: table_keepout; removed torso_front_keepout")
        self.done = True


def main() -> None:
    rclpy.init()
    node = StaticScenePublisher()
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
