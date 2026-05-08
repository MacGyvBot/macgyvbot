"""OpenCV debug window rendering."""

import cv2

from macgyvbot.core.config import HAND_GRASP_WINDOW_NAME, ROBOT_WINDOW_NAME


class DebugWindows:
    def __init__(self, state):
        self.state = state

    def show_robot(self, image):
        cv2.imshow(ROBOT_WINDOW_NAME, image)

    def show_hand_grasp(self):
        if self.state.hand_grasp_image is not None:
            cv2.imshow(HAND_GRASP_WINDOW_NAME, self.state.hand_grasp_image)

    def wait_key(self, delay_ms=1):
        return cv2.waitKey(delay_ms)

    @staticmethod
    def close_all():
        cv2.destroyAllWindows()
