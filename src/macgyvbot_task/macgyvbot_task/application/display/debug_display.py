"""OpenCV debug display helpers for the main task node."""

from __future__ import annotations

import cv2

from macgyvbot_config.ui import HAND_GRASP_WINDOW_NAME, ROBOT_WINDOW_NAME


class DebugDisplay:
    """Show robot and hand-grasp debug frames."""

    def show_robot(self, image):
        cv2.imshow(ROBOT_WINDOW_NAME, image)

    def show_hand_grasp(self, image):
        if image is not None:
            cv2.imshow(HAND_GRASP_WINDOW_NAME, image)

    @staticmethod
    def wait_key(delay_ms=1):
        return cv2.waitKey(delay_ms)

    @staticmethod
    def close():
        cv2.destroyAllWindows()
