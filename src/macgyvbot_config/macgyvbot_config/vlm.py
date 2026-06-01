"""VLM grasp point selection constants."""

GRASP_POINT_MODE_CENTER = "center"
GRASP_POINT_MODE_VLM = "vlm"
GRASP_POINT_MODE_VLM_ONLY_QWEN3B = "vlm_only_qwen3b"
GRASP_POINT_MODE_VLM_ONLY_QWEN7B = "vlm_only_qwen7b"
GRASP_POINT_MODE_VLM_ONLY_SMOL = "vlm_only_smol"
GRASP_POINT_MODE_API = "api"
DEFAULT_GRASP_POINT_MODE = GRASP_POINT_MODE_VLM_ONLY_QWEN3B

VLM_INFERENCE_HISTORY_ENABLED = True
VLM_INFERENCE_HISTORY_DIR = "src/macgyvbot_perception/data/inference_history"

VLM_GRASP_GRID_SIZES = ((3, 3), (4, 4))

VLM_MODEL_SMOL = "HuggingFaceTB__SmolVLM2-2.2B-Instruct"
VLM_MODEL_QWEN3B = "Qwen/Qwen2.5-VL-3B-Instruct"
VLM_MODEL_QWEN7B = "Qwen/Qwen2.5-VL-7B-Instruct"

VLM_ONLY_MODEL_BY_MODE = {
    GRASP_POINT_MODE_VLM_ONLY_SMOL: VLM_MODEL_SMOL,
    GRASP_POINT_MODE_VLM_ONLY_QWEN3B: VLM_MODEL_QWEN3B,
    GRASP_POINT_MODE_VLM_ONLY_QWEN7B: VLM_MODEL_QWEN7B,
}

VLM_ONLY_MODES = tuple(VLM_ONLY_MODEL_BY_MODE.keys())

VLM_GRASP_SERVICE_NAME = "/vlm_grasp"
VLM_SERVICE_WAIT_TIMEOUT_SEC = 2.0
VLM_SERVICE_RESPONSE_TIMEOUT_SEC = 30.0

PCA_YAW_SAM_DEFAULT_CONFIG = {
    "invert_yaw_sign": False,
    "min_mask_area_ratio": 0.01,
    "max_mask_area_ratio": 0.95,
    "min_valid_masks": 2,
    "mask_vote_threshold": 0.5,
    "aggregation_mode": "majority",
    "fallback_to_vlm_yaw": True,
    "morph_kernel_size": 3,
    "morph_open_iterations": 1,
    "morph_close_iterations": 1,
    "min_pca_pixels": 30,
    "sam_positive_point_offsets_px": ((0, 0), (-6, 0), (6, 0), (0, -6), (0, 6)),
    "sam_negative_points_enabled": True,
    "sam_negative_point_margin_ratio": 0.08,
    "sam_negative_point_min_distance_px": 12.0,
    "save_pca_input_mask_image": True,
    "pca_input_mask_dir": "src/macgyvbot_perception/data/yaw_pca",
}
