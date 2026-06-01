"""Planning scene helpers for static drawer collision boundaries."""

from __future__ import annotations

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

from macgyvbot_config.drawer import (
    DRAWER_COLLISION_APPLY_SERVICE,
    DRAWER_COLLISION_BOXES,
    DRAWER_COLLISION_SCENE_TOPICS,
)


class DrawerCollisionSceneManager:
    """Add configured drawer keepout boxes to the MoveIt planning scene."""

    def __init__(
        self,
        node,
        moveit_robot=None,
        boxes=None,
        planning_scene_topics=None,
        apply_service_name=DRAWER_COLLISION_APPLY_SERVICE,
    ):
        self.node = node
        self.moveit_robot = moveit_robot
        self.boxes = list(boxes if boxes is not None else DRAWER_COLLISION_BOXES)
        topics = planning_scene_topics or DRAWER_COLLISION_SCENE_TOPICS
        self._scene_publishers = [
            node.create_publisher(PlanningScene, topic, 10)
            for topic in topics
        ]
        self._apply_client = node.create_client(
            ApplyPlanningScene,
            apply_service_name,
        )
        self._apply_service_name = apply_service_name

    def apply(self, logger=None):
        """Apply drawer boxes locally and publish a scene diff for RViz/move_group."""
        log = logger or self.node.get_logger()
        if not self.boxes:
            _warn(log, "drawer collision boxes are empty; planning scene unchanged")
            return False

        collision_objects, colors = self._build_objects()
        scene = self._build_scene_diff(collision_objects, colors)

        direct_ok = self._apply_to_moveit_py(collision_objects, colors, log)
        self._publish_scene_diff(scene, log)
        service_requested = self._request_apply_scene(scene, log)

        _info(
            log,
            "drawer collision scene update requested: "
            f"objects={len(collision_objects)}, "
            f"moveit_py={direct_ok}, "
            f"apply_service={service_requested}",
        )
        return direct_ok or service_requested

    def _build_objects(self):
        collision_objects = []
        colors = []
        for box in self.boxes:
            collision_objects.append(_make_box_collision_object(box))
            color = _make_object_color(box)
            if color is not None:
                colors.append(color)
        return collision_objects, colors

    @staticmethod
    def _build_scene_diff(collision_objects, colors):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = list(collision_objects)
        scene.object_colors = list(colors)
        return scene

    def _apply_to_moveit_py(self, collision_objects, colors, logger):
        if self.moveit_robot is None:
            return False

        color_by_id = {color.id: color for color in colors}
        try:
            psm = self.moveit_robot.get_planning_scene_monitor()
            with psm.read_write() as planning_scene:
                for collision_object in collision_objects:
                    planning_scene.apply_collision_object(
                        collision_object,
                        color_by_id.get(collision_object.id),
                    )
            return True
        except Exception as exc:
            _warn(
                logger,
                "MoveItPy planning scene에 drawer collision을 직접 적용하지 "
                f"못했습니다: {exc}",
            )
            return False

    def _publish_scene_diff(self, scene, logger):
        for publisher in self._scene_publishers:
            publisher.publish(scene)
        _info(
            logger,
            f"drawer collision PlanningScene diff published "
            f"to {len(self._scene_publishers)} topic(s)",
        )

    def _request_apply_scene(self, scene, logger):
        try:
            if not self._apply_client.wait_for_service(timeout_sec=0.2):
                _warn(
                    logger,
                    "MoveIt apply planning scene service is not ready: "
                    f"{self._apply_service_name}",
                )
                return False

            request = ApplyPlanningScene.Request()
            request.scene = scene
            future = self._apply_client.call_async(request)
            future.add_done_callback(
                lambda done: self._log_apply_result(done, logger)
            )
            return True
        except Exception as exc:
            _warn(
                logger,
                "MoveIt apply planning scene service request failed: "
                f"{exc}",
            )
            return False

    @staticmethod
    def _log_apply_result(future, logger):
        try:
            result = future.result()
        except Exception as exc:
            _warn(logger, f"drawer collision apply service failed: {exc}")
            return

        if getattr(result, "success", False):
            _info(logger, "drawer collision apply service succeeded")
        else:
            _warn(logger, "drawer collision apply service returned success=False")


def _make_box_collision_object(box):
    object_id = str(box["id"])
    frame_id = str(box["frame_id"])
    center_xyz = [float(value) for value in box["center_xyz"]]
    size_xyz = [float(value) for value in box["size_xyz"]]

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = size_xyz

    pose = Pose()
    pose.position.x = center_xyz[0]
    pose.position.y = center_xyz[1]
    pose.position.z = center_xyz[2]
    pose.orientation.w = 1.0

    collision_object = CollisionObject()
    collision_object.id = object_id
    collision_object.header.frame_id = frame_id
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD
    return collision_object


def _make_object_color(box):
    rgba = box.get("color_rgba")
    if rgba is None:
        return None

    color = ObjectColor()
    color.id = str(box["id"])
    color.color = ColorRGBA(
        r=float(rgba[0]),
        g=float(rgba[1]),
        b=float(rgba[2]),
        a=float(rgba[3]),
    )
    return color


def _info(logger, message):
    if logger is not None:
        logger.info(message)


def _warn(logger, message):
    if logger is None:
        return
    warn = getattr(logger, "warn", None) or getattr(logger, "warning", None)
    if warn is not None:
        warn(message)
