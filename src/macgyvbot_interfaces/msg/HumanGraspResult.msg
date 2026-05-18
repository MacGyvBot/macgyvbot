# Human-tool grasp detection result.
# The final grasp decision is true only when ML, depth, and locked-mask checks pass.
# has_* flags distinguish nullable JSON fields from valid zero/empty values.
string state
bool human_grasped_tool
int32 grasp_counter
float32 grasp_score

bool ml_required
bool ml_grasp_confirmed
bool ml_confidence_ok
string ml_raw_state
string ml_stable_state
bool has_ml_confidence
float32 ml_confidence

bool depth_required
bool depth_available
bool depth_grasp_ok
bool depth_grasp_confirmed
bool has_tool_depth_mm
float32 tool_depth_mm
bool has_min_hand_tool_depth_diff_mm
float32 min_hand_tool_depth_diff_mm
int32 depth_contact_count

bool mask_locked
bool locked_mask_grasp_ok
bool mask_contact_confirmed
bool mask_proximity_ok
bool mask_near_or_contact
string mask_source
int32 mask_contact_count
bool has_min_landmark_to_tool_distance
float32 min_landmark_to_tool_distance

bool has_tool_label
string tool_label
bool has_tool_confidence
float32 tool_confidence

# ROI is [x1, y1, x2, y2] in image pixels.
bool has_tool_roi
int32[4] tool_roi

bool has_hand_pixel
int32 hand_u
int32 hand_v
string hand_pixel_source

bool has_active_hand_index
int32 active_hand_index
bool has_active_handedness
string active_handedness
string payload_json
