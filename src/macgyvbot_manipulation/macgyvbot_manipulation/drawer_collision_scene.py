"""Planning scene helpers for static drawer collision boundaries."""

from __future__ import annotations

import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

from macgyvbot_config.drawer import (
    DRAWER_COLLISION_APPLY_SERVICE,
    DRAWER_COLLISION_BOX_PROFILES,
    DRAWER_COLLISION_BOXES,
    DRAWER_COLLISION_DEFAULT_PROFILE,
    DRAWER_COLLISION_SCENE_TOPICS,
    DRAWER_COLLISION_SCENE_KEY_PROFILES,
)
from macgyvbot_config.structured_logging import format_structured_log

_LOG_SVC = "manipulation"
_LOG_PIPE = "drawer_collision_scene"
_LOG_STEP = "drawer_scene"


class DrawerCollisionSceneManager:
    """Add configured drawer keepout boxes to the MoveIt planning scene."""

    def __init__(
        self,
        node,
        moveit_robot=None,
        boxes=None,
        profiles=None,
        profile_by_scene_key=None,
        default_profile=DRAWER_COLLISION_DEFAULT_PROFILE,
        planning_scene_topics=None,
        apply_service_name=DRAWER_COLLISION_APPLY_SERVICE,
    ):
        self.node = node
        self.moveit_robot = moveit_robot
        self.default_profile = str(default_profile)
        if profiles is None:
            profiles = (
                {self.default_profile: list(boxes)}
                if boxes is not None
                else DRAWER_COLLISION_BOX_PROFILES
            )
        self.profiles = {
            str(profile_name): list(profile_boxes)
            for profile_name, profile_boxes in profiles.items()
        }
        if self.default_profile not in self.profiles and self.profiles:
            self.default_profile = next(iter(self.profiles))
        self.boxes = list(
            self.profiles.get(self.default_profile, DRAWER_COLLISION_BOXES)
        )
        self.profile_by_scene_key = {
            str(scene_key): str(profile_name)
            for scene_key, profile_name in (
                profile_by_scene_key or DRAWER_COLLISION_SCENE_KEY_PROFILES
            ).items()
        }
        self._known_object_ids = {
            str(box["id"])
            for profile_boxes in self.profiles.values()
            for box in profile_boxes
        }
        self._object_frame_ids = {
            str(box["id"]): str(box["frame_id"])
            for profile_boxes in self.profiles.values()
            for box in profile_boxes
        }
        self._applied_to_moveit_py = False
        self._applied_profile = None
        self._applied_object_ids = set()
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

    def apply(self, logger=None, publish=True, request_service=True, profile=None):
        """Apply drawer boxes locally and publish a scene diff for RViz/move_group."""
        log = logger or self.node.get_logger()
        profile_name = self._normalize_profile(profile, log)
        boxes = self._boxes_for_profile(profile_name)
        if not boxes:
            _warn(
                log,
                "drawer collision boxes are empty; "
                f"profile={profile_name}, planning scene unchanged",
            )
            return False

        collision_objects, colors, active_object_ids = self._build_objects(boxes)
        removal_objects = self._build_removal_objects(active_object_ids)
        scene_objects = collision_objects + removal_objects
        scene = self._build_scene_diff(scene_objects, colors)

        direct_ok = self._apply_to_moveit_py(scene_objects, colors, log)
        self._applied_to_moveit_py = bool(direct_ok)
        if direct_ok:
            self._applied_profile = profile_name
            self._applied_object_ids = set(active_object_ids)
        if publish:
            self._publish_scene_diff(scene, log)
        service_requested = False
        if request_service:
            service_requested = self._request_apply_scene(scene, log)

        _info(
            log,
            "drawer collision scene update requested: "
            f"profile={profile_name}, "
            f"active_objects={len(collision_objects)}, "
            f"removed_objects={len(removal_objects)}, "
            f"moveit_py={direct_ok}, "
            f"published={publish}, "
            f"apply_service={service_requested}",
        )
        return direct_ok or service_requested

    def is_ready(self, profile=None):
        """Return whether drawer boxes were applied to the local MoveItPy scene."""
        profile_name = self._normalize_profile(profile)
        active_object_ids = self._object_ids_for_profile(profile_name)
        return (
            bool(active_object_ids)
            and self._applied_to_moveit_py
            and self._applied_profile == profile_name
            and self._applied_object_ids == active_object_ids
        )

    def profile_for_scene_key(self, scene_key):
        if scene_key is None:
            return self.default_profile
        return self.profile_by_scene_key.get(
            str(scene_key),
            self.default_profile,
        )

    def ensure_ready_for_scene_key(
        self,
        scene_key,
        logger=None,
        attempts=1,
        retry_delay_sec=0.0,
        refresh=True,
    ):
        return self.ensure_ready(
            logger=logger,
            attempts=attempts,
            retry_delay_sec=retry_delay_sec,
            refresh=refresh,
            profile=self.profile_for_scene_key(scene_key),
        )

    def ensure_ready(
        self,
        logger=None,
        attempts=1,
        retry_delay_sec=0.0,
        refresh=True,
        profile=None,
    ):
        """Ensure drawer boxes are present before a motion plan is requested."""
        log = logger or self.node.get_logger()
        profile_name = self._normalize_profile(profile, log)
        boxes = self._boxes_for_profile(profile_name)
        if not boxes:
            _warn(
                log,
                "drawer collision boxes are empty; "
                f"profile={profile_name}, motion is not safe",
            )
            return False
        if not refresh and self.is_ready(profile_name):
            return True

        attempts = max(1, int(attempts))
        sync_external_scene = self._needs_external_scene_sync(profile_name)
        for attempt_index in range(attempts):
            sync_this_attempt = sync_external_scene and attempt_index == 0
            self.apply(
                log,
                publish=sync_this_attempt,
                request_service=sync_this_attempt,
                profile=profile_name,
            )
            if self.is_ready(profile_name):
                return True
            if attempt_index + 1 < attempts and retry_delay_sec > 0.0:
                time.sleep(float(retry_delay_sec))

        _warn(
            log,
            "drawer collision scene is not ready; "
            f"profile={profile_name}, planning is blocked",
        )
        return False

    def _normalize_profile(self, profile, logger=None):
        profile_name = str(profile or self.default_profile)
        if profile_name in self.profiles:
            return profile_name

        _warn(
            logger,
            f"unknown drawer collision profile={profile_name}; "
            f"using default profile={self.default_profile}",
        )
        return self.default_profile

    def _boxes_for_profile(self, profile):
        return list(self.profiles.get(profile, []))

    def _object_ids_for_profile(self, profile):
        return {str(box["id"]) for box in self._boxes_for_profile(profile)}

    def _needs_external_scene_sync(self, profile):
        return (
            self._applied_profile != profile
            or self._applied_object_ids != self._object_ids_for_profile(profile)
        )

    def _build_objects(self, boxes):
        collision_objects = []
        colors = []
        object_ids = set()
        for box in boxes:
            collision_objects.append(_make_box_collision_object(box))
            object_ids.add(str(box["id"]))
            color = _make_object_color(box)
            if color is not None:
                colors.append(color)
        return collision_objects, colors, object_ids

    def _build_removal_objects(self, active_object_ids):
        inactive_object_ids = self._known_object_ids - set(active_object_ids)
        return [
            _make_remove_collision_object(
                object_id,
                self._object_frame_ids.get(object_id, ""),
            )
            for object_id in sorted(inactive_object_ids)
        ]

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


def _make_remove_collision_object(object_id, frame_id):
    collision_object = CollisionObject()
    collision_object.id = str(object_id)
    collision_object.header.frame_id = str(frame_id)
    collision_object.operation = CollisionObject.REMOVE
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
    if logger is None:
        return
    info = getattr(_raw_logger(logger), "info", None)
    if info is not None:
        info(_format_log("info", message))


def _warn(logger, message):
    if logger is None:
        return
    raw_logger = _raw_logger(logger)
    warn = getattr(raw_logger, "warn", None) or getattr(
        raw_logger,
        "warning",
        None,
    )
    if warn is not None:
        warn(_format_log("warn", message))


def _format_log(level, message):
    return format_structured_log(
        svc=_LOG_SVC,
        pipe=_LOG_PIPE,
        step=_LOG_STEP,
        event=level,
        msg=message,
    )


def _raw_logger(logger):
    return getattr(logger, "_logger", logger)
