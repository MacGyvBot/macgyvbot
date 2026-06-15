import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
package_path = REPO_ROOT / "src" / "macgyvbot_config"
if str(package_path) not in sys.path:
    sys.path.insert(0, str(package_path))

from macgyvbot_config.hand_grasp import HAND_GRASP_YOLO_CONFIDENCE
from macgyvbot_config.models import YOLO_CONFIDENCE_THRESHOLD


def test_yolo_confidence_defaults_to_runtime_threshold():
    assert YOLO_CONFIDENCE_THRESHOLD == 0.40
    assert HAND_GRASP_YOLO_CONFIDENCE == YOLO_CONFIDENCE_THRESHOLD
