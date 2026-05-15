"""Topic-driven square handover search helpers (single-package, non-blocking)."""

import json
import threading
import time
import uuid

import rclpy
from std_msgs.msg import String
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import make_safe_pose

from macgyvbot.config.config import (
    ALLOW_SEARCH_VERTEX_FALLBACK_FOR_HANDOVER,
    BASE_FRAME,
    HAND_GRASP_TIMEOUT_SEC,
    HAND_POSE_WAIT_AFTER_DETECTION_SEC,
    HANDOVER_SEARCH_TIMEOUT_SEC,
    SAFE_Z,
    SEQUENCE_WAIT_POLL_SEC,
)
from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import (
    clamp_to_safe_workspace,
)

HANDOVER_SEARCH_REQUEST_TOPIC = "/handover_square_search/request"
HANDOVER_SEARCH_RESULT_TOPIC = "/handover_square_search/result"
CANDIDATE_SUCCESS_STREAK = 6
VERTEX_DWELL_SEC = 3.0
HANDOVER_SEARCH_MIN_TIMEOUT_SEC = 45.0
HANDOVER_SEARCH_CLIENT_EXTRA_SEC = 8.0
HANDOVER_SEARCH_PER_VERTEX_MOVE_BUDGET_SEC = 4.0


class SquareHandoverSearchServer:
    def __init__(self, node, motion_controller, state):
        self.node = node
        self.motion = motion_controller
        self.state = state
        self._result_pub = self.node.create_publisher(
            String,
            HANDOVER_SEARCH_RESULT_TOPIC,
            10,
        )
        self.node.create_subscription(
            String,
            HANDOVER_SEARCH_REQUEST_TOPIC,
            self._on_request,
            10,
        )
        self._lock = threading.Lock()
        self._busy = False

    def _on_request(self, msg):
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            self.node.get_logger().warn(f"handover search 요청 JSON 파싱 실패: {msg.data}")
            return

        with self._lock:
            if self._busy:
                self._publish_result(
                    request_id=req.get("request_id", ""),
                    success=False,
                    found=False,
                    x=float(req.get("target_x", 0.0)),
                    y=float(req.get("target_y", 0.0)),
                    z=float(req.get("travel_z", SAFE_Z)),
                    frame_id="world",
                    source="none",
                    message="이미 사각형 탐색을 수행 중입니다.",
                    failure_reason="search_busy",
                    vertex_x=float(req.get("target_x", 0.0)),
                    vertex_y=float(req.get("target_y", 0.0)),
                    vertex_z=float(req.get("travel_z", SAFE_Z)),
                    hand_pose_available=False,
                    elapsed_sec=0.0,
                    visited_count=0,
                    effective_timeout_sec=0.0,
                )
                return
            self._busy = True

        threading.Thread(
            target=self._run_search_job,
            args=(req,),
            daemon=True,
        ).start()

    def _run_search_job(self, req):
        request_id = req.get("request_id") or ""
        target_x = float(req["target_x"])
        target_y = float(req["target_y"])
        travel_z = float(req["travel_z"])
        ori = req["ori"]
        timeout_sec = float(req.get("timeout_sec", HAND_GRASP_TIMEOUT_SEC))
        logger = self.node.get_logger()

        search_offsets_m = self._build_search_offsets()
        required_timeout = self._required_timeout_sec(len(search_offsets_m))
        effective_timeout = max(
            timeout_sec,
            HANDOVER_SEARCH_MIN_TIMEOUT_SEC,
            required_timeout,
        )
        logger.info(
            "사각형 탐색 timeout 보정: "
            f"requested={timeout_sec:.1f}s, required={required_timeout:.1f}s, "
            f"effective={effective_timeout:.1f}s"
        )

        try:
            (
                found,
                found_x,
                found_y,
                found_z,
                found_frame_id,
                visited_count,
                elapsed_sec,
                failure_reason,
                source,
                vertex_x,
                vertex_y,
                vertex_z,
                hand_pose_available,
            ) = self._search(
                target_x,
                target_y,
                travel_z,
                ori,
                effective_timeout,
                search_offsets_m,
                logger,
            )

            if found:
                message = "손 탐지 성공"
            elif failure_reason == "initial_search_pose_failed":
                message = "초기 탐색 자세 IK/Planning 실패"
            elif failure_reason == "hand_pose_missing_xyz":
                message = "손 감지는 되었지만 3D 좌표(x,y,z)가 없어 전달 실패"
            elif failure_reason == "hand_pose_unsupported_frame":
                message = "손 위치 frame이 planning frame이 아니어서 전달 실패"
            else:
                message = "손 미탐지. 기본 위치 유지"

            self._publish_result(
                request_id=request_id,
                success=True,
                found=found,
                x=found_x,
                y=found_y,
                z=found_z,
                frame_id=found_frame_id,
                source=source,
                message=message,
                failure_reason=failure_reason,
                vertex_x=vertex_x,
                vertex_y=vertex_y,
                vertex_z=vertex_z,
                hand_pose_available=hand_pose_available,
                elapsed_sec=elapsed_sec,
                visited_count=visited_count,
                effective_timeout_sec=effective_timeout,
            )
        except Exception as exc:  # pragma: no cover
            logger.error(f"사각형 탐색 실행 예외: {exc}")
            self._publish_result(
                request_id=request_id,
                success=False,
                found=False,
                x=target_x,
                y=target_y,
                z=travel_z,
                frame_id="world",
                source="none",
                message=f"사각형 탐색 실패: {exc}",
                failure_reason="search_exception",
                vertex_x=target_x,
                vertex_y=target_y,
                vertex_z=travel_z,
                hand_pose_available=False,
                elapsed_sec=0.0,
                visited_count=0,
                effective_timeout_sec=effective_timeout,
            )
        finally:
            with self._lock:
                self._busy = False

    @staticmethod
    def _build_search_offsets():
        offset = 0.10
        return [
            ("center", 0.0, 0.0),
            ("left_down", -offset, -offset),
            ("left_up", -offset, offset),
            ("right_up", offset, offset),
            ("right_down", offset, -offset),
            ("center_return", 0.0, 0.0),
        ]

    @staticmethod
    def _required_timeout_sec(vertex_count):
        return vertex_count * (
            VERTEX_DWELL_SEC + HANDOVER_SEARCH_PER_VERTEX_MOVE_BUDGET_SEC
        )

    @staticmethod
    def _remaining_sec(start_time, effective_timeout):
        return effective_timeout - (time.monotonic() - start_time)

    @staticmethod
    def _extract_xyz_from_result(result):
        position = result.get("position")
        if isinstance(position, dict) and all(k in position for k in ("x", "y", "z")):
            return float(position["x"]), float(position["y"]), float(position["z"])
        if all(k in result for k in ("x", "y", "z")):
            return float(result["x"]), float(result["y"]), float(result["z"])
        if all(k in result for k in ("hand_x", "hand_y", "hand_z")):
            return float(result["hand_x"]), float(result["hand_y"]), float(result["hand_z"])
        for key in ("point", "hand_camera"):
            v = result.get(key)
            if isinstance(v, dict) and all(k in v for k in ("x", "y", "z")):
                return float(v["x"]), float(v["y"]), float(v["z"])
        return None

    @staticmethod
    def _extract_frame_id(result):
        position = result.get("position")
        if isinstance(position, dict) and isinstance(position.get("frame_id"), str):
            return position["frame_id"]
        if isinstance(result.get("frame_id"), str):
            return result["frame_id"]
        for key in ("hand_camera", "point"):
            v = result.get(key)
            if isinstance(v, dict) and isinstance(v.get("frame_id"), str):
                return v["frame_id"]
        header = result.get("header")
        if isinstance(header, dict) and isinstance(header.get("frame_id"), str):
            return header["frame_id"]
        return ""

    @staticmethod
    def _has_hand_observation(result):
        if isinstance(result.get("hand_pixel"), dict):
            return True
        if result.get("active_hand_index") is not None:
            return True
        if isinstance(result.get("position"), dict):
            return True
        return False

    def _extract_hand_pose(self, fallback_x, fallback_y, fallback_z, logger):
        result = self.state.last_grasp_result or {}
        logger.info(f"last_grasp_result 원본={result}")

        xyz = self._extract_xyz_from_result(result)
        frame_id = self._extract_frame_id(result) or "world"
        if xyz is None:
            logger.warn(
                "손 감지 결과에 3D 좌표가 없습니다. "
                f"keys={list(result.keys())}, frame_id={frame_id}, result={result}. "
                "expected keys: x/y/z, hand_x/hand_y/hand_z, position{x,y,z}, point{x,y,z}"
            )
            if ALLOW_SEARCH_VERTEX_FALLBACK_FOR_HANDOVER:
                logger.warn("손 좌표가 없어 탐색 vertex fallback을 사용합니다.")
                return (
                    True,
                    fallback_x,
                    fallback_y,
                    fallback_z,
                    "world",
                    "search_vertex",
                    "ok",
                )
            return False, fallback_x, fallback_y, fallback_z, frame_id, "none", "missing_xyz"

        hx, hy, hz = xyz
        if frame_id in ("world", BASE_FRAME):
            logger.info(
                "hand pose selected: "
                f"source=hand_pose, frame={frame_id}, xyz=({hx:.3f},{hy:.3f},{hz:.3f})"
            )
            return True, hx, hy, hz, frame_id, "hand_pose", "ok"

        logger.warn(
            "hand pose frame is not accepted for planning: "
            f"frame={frame_id}, xyz=({hx:.3f},{hy:.3f},{hz:.3f}). "
            "Camera-frame hand positions must be converted before publishing."
        )
        return False, fallback_x, fallback_y, fallback_z, frame_id, "none", "unsupported_frame"

    def _search(self, target_x, target_y, travel_z, ori, effective_timeout, search_offsets_m, logger):
        start_time = time.monotonic()
        search_z = float(travel_z)
        visited_count = 0
        attempted_count = 0
        all_moves_failed = True
        last_candidate_log_time = 0.0
        hand_pose_missing_after_detection = False

        logger.info(
            "사각형 탐색 시작: "
            f"handoff_pose_z={travel_z:.3f}, search_z={search_z:.3f} (추가 상승 없음)"
        )

        total_vertices = len(search_offsets_m)
        last_hand_pose_reason = ""
        while rclpy.ok() and self._remaining_sec(start_time, effective_timeout) > 0.0:
            for vertex_index, (vertex_name, dx, dy) in enumerate(search_offsets_m, start=1):
                remaining_before_move = self._remaining_sec(start_time, effective_timeout)
                if remaining_before_move <= HANDOVER_SEARCH_PER_VERTEX_MOVE_BUDGET_SEC:
                    elapsed = time.monotonic() - start_time
                    return (
                        False,
                        target_x,
                        target_y,
                        search_z,
                        "world",
                        visited_count,
                        elapsed,
                        "handover_timeout",
                        "none",
                        target_x,
                        target_y,
                        search_z,
                        hand_pose_missing_after_detection,
                    )

                raw_x = target_x + dx
                raw_y = target_y + dy
                clamped_x, clamped_y, clamped_z = clamp_to_safe_workspace(raw_x, raw_y, search_z, logger)
                logger.info(
                    "square vertex "
                    f"{vertex_name}({vertex_index}/{total_vertices}): "
                    f"raw=({raw_x:.3f},{raw_y:.3f},{search_z:.3f}), "
                    f"clamped=({clamped_x:.3f},{clamped_y:.3f},{clamped_z:.3f}), "
                    f"offset=({dx:.3f},{dy:.3f}), vertex_z={search_z:.3f}"
                )

                attempted_count += 1
                ok = self.motion.plan_and_execute(
                    logger,
                    pose_goal=make_safe_pose(clamped_x, clamped_y, search_z, ori, logger),
                )
                if not ok:
                    logger.warn(
                        "사각형 탐색 꼭짓점 이동 실패: "
                        f"x={clamped_x:.3f}, y={clamped_y:.3f}, z={search_z:.3f}. 다음 지점으로 진행합니다."
                    )
                    continue

                all_moves_failed = False
                visited_count += 1
                check_start = time.monotonic()
                while time.monotonic() - check_start < VERTEX_DWELL_SEC:
                    last = self.state.last_grasp_result or {}
                    if not self._has_hand_observation(last):
                        time.sleep(min(SEQUENCE_WAIT_POLL_SEC, 0.1))
                        continue

                    (
                        ok_pose,
                        hx,
                        hy,
                        hz,
                        hframe,
                        source,
                        pose_reason,
                    ) = self._extract_hand_pose(clamped_x, clamped_y, search_z, logger)
                    if ok_pose:
                        logger.info(
                            "search vertex 대비 hand pose 차이: "
                            f"vertex=({clamped_x:.3f},{clamped_y:.3f},{search_z:.3f}), "
                            f"hand=({hx:.3f},{hy:.3f},{hz:.3f}), "
                            f"delta=({hx - clamped_x:.3f},{hy - clamped_y:.3f},{hz - search_z:.3f})"
                        )
                        elapsed = time.monotonic() - start_time
                        return (
                            True,
                            hx,
                            hy,
                            hz,
                            hframe,
                            visited_count,
                            elapsed,
                            "",
                            source,
                            clamped_x,
                            clamped_y,
                            search_z,
                            True,
                        )

                    last_hand_pose_reason = pose_reason
                    hand_pose_missing_after_detection = True
                    now = time.monotonic()
                    if now - last_candidate_log_time >= 0.7:
                        logger.info(
                            "손 중심 관측은 되었지만 3D 좌표 보강 실패: "
                            f"reason={pose_reason}"
                        )
                        last_candidate_log_time = now
                    time.sleep(min(SEQUENCE_WAIT_POLL_SEC, 0.1))

            if attempted_count > 0 and all_moves_failed:
                elapsed = time.monotonic() - start_time
                return (
                    False,
                    target_x,
                    target_y,
                    search_z,
                    "world",
                    visited_count,
                    elapsed,
                    "all_vertex_moves_failed",
                    "none",
                    target_x,
                    target_y,
                    search_z,
                    False,
                )

        elapsed = time.monotonic() - start_time
        if hand_pose_missing_after_detection:
            if last_hand_pose_reason == "missing_xyz":
                failure_reason = "hand_pose_missing_xyz"
            elif last_hand_pose_reason == "unsupported_frame":
                failure_reason = "hand_pose_unsupported_frame"
            else:
                failure_reason = "hand_pose_unavailable"
        else:
            failure_reason = "handover_timeout"
        return (
            False,
            target_x,
            target_y,
            search_z,
            "world",
            visited_count,
            elapsed,
            failure_reason,
            "none",
            target_x,
            target_y,
            search_z,
            hand_pose_missing_after_detection,
        )

    def _publish_result(
        self,
        request_id,
        success,
        found,
        x,
        y,
        z,
        frame_id,
        source,
        message,
        failure_reason,
        vertex_x,
        vertex_y,
        vertex_z,
        hand_pose_available,
        elapsed_sec,
        visited_count,
        effective_timeout_sec,
    ):
        payload = {
            "request_id": request_id,
            "success": bool(success),
            "found": bool(found),
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "frame_id": frame_id,
            "source": source,
            "message": message,
            "failure_reason": failure_reason,
            "vertex_x": float(vertex_x),
            "vertex_y": float(vertex_y),
            "vertex_z": float(vertex_z),
            "hand_pose_available": bool(hand_pose_available),
            "elapsed_sec": float(elapsed_sec),
            "visited_count": int(visited_count),
            "effective_timeout_sec": float(effective_timeout_sec),
        }
        self._result_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


class SquareHandoverSearchClient:
    def __init__(self, node):
        self.node = node
        self._request_pub = self.node.create_publisher(
            String,
            HANDOVER_SEARCH_REQUEST_TOPIC,
            10,
        )
        self.node.create_subscription(
            String,
            HANDOVER_SEARCH_RESULT_TOPIC,
            self._on_result,
            10,
        )
        self._results = {}
        self._lock = threading.Lock()
        self.last_result = None

    def _on_result(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.node.get_logger().warn(f"handover search 결과 JSON 파싱 실패: {msg.data}")
            return
        request_id = result.get("request_id")
        if not request_id:
            return
        with self._lock:
            self._results[request_id] = result

    def request_search(self, target_x, target_y, travel_z, ori, logger, timeout_sec=None):
        timeout_sec = float(timeout_sec or HANDOVER_SEARCH_TIMEOUT_SEC)
        search_offsets_m = SquareHandoverSearchServer._build_search_offsets()
        required_timeout = SquareHandoverSearchServer._required_timeout_sec(len(search_offsets_m))
        effective_timeout = max(
            timeout_sec,
            HANDOVER_SEARCH_MIN_TIMEOUT_SEC,
            required_timeout,
        )

        request_id = str(uuid.uuid4())
        payload = {
            "request_id": request_id,
            "target_x": float(target_x),
            "target_y": float(target_y),
            "travel_z": float(travel_z),
            "ori": ori,
            "timeout_sec": effective_timeout,
        }
        self._request_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        logger.info(
            "사각형 탐색 노드에 요청을 전송했습니다. "
            f"request_id={request_id}, requested_timeout={timeout_sec:.1f}s, "
            f"effective_timeout={effective_timeout:.1f}s"
        )

        wait_budget_sec = max(timeout_sec, HANDOVER_SEARCH_MIN_TIMEOUT_SEC) + HANDOVER_SEARCH_CLIENT_EXTRA_SEC
        deadline = time.monotonic() + wait_budget_sec
        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                result = self._results.pop(request_id, None)
            if result is not None:
                self.last_result = result
                if not result.get("success", False):
                    logger.warn(f"사각형 탐색 노드 실패 응답: {result.get('message', '')}")
                    return False, float(target_x), float(target_y), float(travel_z), "world", "none"
                logger.info(
                    "사각형 탐색 결과 수신: "
                    f"request_id={request_id}, found={result.get('found')}, "
                    f"source={result.get('source')}, frame={result.get('frame_id')}, "
                    f"visited={result.get('visited_count')}, "
                    f"elapsed={result.get('elapsed_sec')}, "
                    f"effective_timeout={result.get('effective_timeout_sec')}"
                )
                return (
                    bool(result.get("found", False)),
                    float(result.get("x", target_x)),
                    float(result.get("y", target_y)),
                    float(result.get("z", travel_z)),
                    str(result.get("frame_id", "world")),
                    str(result.get("source", "none")),
                )
            time.sleep(min(SEQUENCE_WAIT_POLL_SEC, 0.1))

        logger.warn(
            "사각형 탐색 노드 응답 타임아웃. 기본 위치를 사용합니다. "
            f"request_id={request_id}, requested_timeout={timeout_sec:.1f}s, "
            f"wait_budget={wait_budget_sec:.1f}s"
        )
        return False, float(target_x), float(target_y), float(travel_z), "world", "none"
