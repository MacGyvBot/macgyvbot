"""Planning scene helpers for RG2 internal self-collision filtering."""

from __future__ import annotations

import time

from moveit_msgs.msg import (
    AllowedCollisionEntry,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

from macgyvbot_config.gripper import (
    DEFAULT_APPLY_PLANNING_SCENE_SERVICE,
    DEFAULT_GET_PLANNING_SCENE_SERVICE,
    DEFAULT_PLANNING_SCENE_TOPICS,
    DEFAULT_PLANNING_SCENE_SERVICE_TIMEOUT_SEC,
    DEFAULT_SCENE_SETTLE_SEC,
    RG2_ALLOWED_COLLISION_PAIRS,
)
from macgyvbot_config.structured_logging import format_structured_log
from macgyvbot_config.timing import ROS_FUTURE_POLL_SEC

_LOG_SVC = "manipulation"
_LOG_PIPE = "gripper_collision_scene"
_LOG_STEP = "gripper_acm"


class GripperSelfCollisionManager:
    """Allow only RG2 internal link-link collisions in MoveIt's ACM."""

    def __init__(
        self,
        node,
        moveit_robot=None,
        allowed_pairs=None,
        planning_scene_topics=None,
        get_service_name=DEFAULT_GET_PLANNING_SCENE_SERVICE,
        apply_service_name=DEFAULT_APPLY_PLANNING_SCENE_SERVICE,
    ):
        self.node = node
        self.moveit_robot = moveit_robot
        self.allowed_pairs = _normalize_pairs(
            allowed_pairs or RG2_ALLOWED_COLLISION_PAIRS
        )
        topics = planning_scene_topics or DEFAULT_PLANNING_SCENE_TOPICS
        self._scene_publishers = [
            node.create_publisher(PlanningScene, topic, 10)
            for topic in topics
        ]
        self._get_client = node.create_client(
            GetPlanningScene,
            get_service_name,
        )
        self._apply_client = node.create_client(
            ApplyPlanningScene,
            apply_service_name,
        )
        self._get_service_name = get_service_name
        self._apply_service_name = apply_service_name

    def apply(
        self,
        logger=None,
        timeout_sec=DEFAULT_PLANNING_SCENE_SERVICE_TIMEOUT_SEC,
    ):
        """Patch RG2 internal pairs in the local and shared planning scene ACM."""
        log = logger or self.node.get_logger()
        local_ok, local_changed = self._apply_to_moveit_py(log)

        response = self._request_current_scene(log, timeout_sec)
        if response is None:
            if local_ok:
                _info(
                    log,
                    "RG2 self-collision ACM patched in local MoveItPy scene: "
                    f"pairs={len(self.allowed_pairs)}, changed={local_changed}",
                )
            return local_ok

        acm = response.scene.allowed_collision_matrix
        changed = _allow_collision_pairs(acm, self.allowed_pairs)

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = acm

        self._publish_scene_diff(scene, log)
        time.sleep(DEFAULT_SCENE_SETTLE_SEC)
        service_ok = self._request_apply_scene(scene, log, timeout_sec)
        if service_ok:
            self._publish_scene_diff(scene, log)
            time.sleep(DEFAULT_SCENE_SETTLE_SEC)

        _info(
            log,
            "RG2 self-collision ACM patch requested: "
            f"pairs={len(self.allowed_pairs)}, "
            f"local_moveit_py={local_ok}, local_changed={local_changed}, "
            f"service_changed={changed}, "
            f"apply_service={service_ok}",
        )
        return local_ok if self.moveit_robot is not None else service_ok

    def _apply_to_moveit_py(self, logger):
        if self.moveit_robot is None:
            return False, 0

        try:
            psm = self.moveit_robot.get_planning_scene_monitor()
            with psm.read_write() as planning_scene:
                acm = planning_scene.allowed_collision_matrix
                changed = _allow_core_collision_pairs(acm, self.allowed_pairs)
                _set_core_collision_matrix_if_supported(planning_scene, acm)

            with psm.read_only() as planning_scene:
                acm = planning_scene.allowed_collision_matrix
                if _core_collision_pairs_allowed(acm, self.allowed_pairs):
                    return True, changed
        except Exception as exc:
            _warn(
                logger,
                "MoveItPy local planning scene에 RG2 self-collision ACM을 "
                f"직접 적용하지 못했습니다: {exc}",
            )
            return False, 0

        _warn(
            logger,
            "MoveItPy local planning scene RG2 self-collision ACM 적용을 "
            "확인하지 못했습니다.",
        )
        return False, 0

    def _request_current_scene(self, logger, timeout_sec):
        if not self._get_client.wait_for_service(timeout_sec=timeout_sec):
            _warn(
                logger,
                "MoveIt get planning scene service is not ready: "
                f"{self._get_service_name}",
            )
            return None

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        future = self._get_client.call_async(request)
        return _wait_for_future(
            future,
            timeout_sec,
            logger,
            f"{self._get_service_name} response",
        )

    def _request_apply_scene(self, scene, logger, timeout_sec):
        if not self._apply_client.wait_for_service(timeout_sec=timeout_sec):
            _warn(
                logger,
                "MoveIt apply planning scene service is not ready: "
                f"{self._apply_service_name}",
            )
            return False

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._apply_client.call_async(request)
        result = _wait_for_future(
            future,
            timeout_sec,
            logger,
            f"{self._apply_service_name} response",
        )
        if result is None:
            return False
        if getattr(result, "success", False):
            _info(logger, "RG2 self-collision ACM apply service succeeded")
            return True

        _warn(logger, "RG2 self-collision ACM apply service returned success=False")
        return False

    def _publish_scene_diff(self, scene, logger):
        for publisher in self._scene_publishers:
            publisher.publish(scene)
        _info(
            logger,
            "RG2 self-collision ACM PlanningScene diff published "
            f"to {len(self._scene_publishers)} topic(s)",
        )


def _allow_collision_pairs(acm, pairs):
    names = list(acm.entry_names)
    rows = [list(entry.enabled) for entry in acm.entry_values]
    _normalize_matrix(names, rows)

    changed = 0
    for left_name, right_name in pairs:
        left_index = _ensure_name(names, rows, left_name)
        right_index = _ensure_name(names, rows, right_name)
        if not rows[left_index][right_index]:
            changed += 1
        rows[left_index][right_index] = True
        rows[right_index][left_index] = True

    acm.entry_names = list(names)
    acm.entry_values = [_make_entry(row) for row in rows]
    return changed


def _allow_core_collision_pairs(acm, pairs):
    changed = 0
    for left_name, right_name in pairs:
        was_allowed = _get_core_collision_entry(acm, left_name, right_name)
        acm.set_entry(left_name, right_name, True)
        if was_allowed is not True:
            changed += 1
    return changed


def _core_collision_pairs_allowed(acm, pairs):
    for left_name, right_name in pairs:
        if _get_core_collision_entry(acm, left_name, right_name) is not True:
            return False
    return True


def _get_core_collision_entry(acm, left_name, right_name):
    try:
        result = acm.get_entry(left_name, right_name)
    except Exception:
        return None
    if isinstance(result, (tuple, list)):
        if not result:
            return None
        return bool(result[0])
    return bool(result)


def _set_core_collision_matrix_if_supported(planning_scene, acm):
    try:
        planning_scene.allowed_collision_matrix = acm
        return True
    except Exception:
        return False


def _normalize_matrix(names, rows):
    while len(rows) < len(names):
        rows.append([])
    del rows[len(names) :]

    width = len(names)
    for row in rows:
        if len(row) < width:
            row.extend([False] * (width - len(row)))
        del row[width:]


def _ensure_name(names, rows, name):
    if name in names:
        return names.index(name)

    names.append(name)
    for row in rows:
        row.append(False)
    rows.append([False] * len(names))
    return len(names) - 1


def _make_entry(row):
    entry = AllowedCollisionEntry()
    entry.enabled = [bool(value) for value in row]
    return entry


def _normalize_pairs(pairs):
    normalized = []
    seen = set()
    for left_name, right_name in pairs:
        left_name = str(left_name).strip()
        right_name = str(right_name).strip()
        if not left_name or not right_name or left_name == right_name:
            continue
        key = tuple(sorted((left_name, right_name)))
        if key in seen:
            continue
        seen.add(key)
        normalized.append((left_name, right_name))
    return tuple(normalized)


def _wait_for_future(future, timeout_sec, logger, label):
    deadline = time.monotonic() + float(timeout_sec)
    while not future.done():
        if time.monotonic() >= deadline:
            _warn(logger, f"Timed out waiting for {label}")
            return None
        time.sleep(ROS_FUTURE_POLL_SEC)

    try:
        return future.result()
    except Exception as exc:
        _warn(logger, f"Failed waiting for {label}: {exc}")
        return None


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
