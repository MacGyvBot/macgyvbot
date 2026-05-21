"""OpenCV debug display helpers for the main task node."""

from __future__ import annotations

import cv2

from macgyvbot_config.ui import HAND_GRASP_WINDOW_NAME, ROBOT_WINDOW_NAME


class DebugDisplay:
    """Show robot and hand-grasp debug frames when enabled."""

    def __init__(self, enabled=True):
        self.enabled = bool(enabled)

    def show_robot(self, image):
        if not self.enabled:
            return
        cv2.imshow(ROBOT_WINDOW_NAME, image)

    def show_hand_grasp(self, image):
        if not self.enabled:
            return
        if image is not None:
            cv2.imshow(HAND_GRASP_WINDOW_NAME, image)

    def wait_key(self, delay_ms=1):
        if not self.enabled:
            return -1
        return cv2.waitKey(delay_ms)

    def close(self):
        if not self.enabled:
            return
        cv2.destroyAllWindows()
