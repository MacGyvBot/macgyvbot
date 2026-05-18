# Status of the currently locked tool mask used by hand grasp detection.
# payload_json preserves the complete existing JSON lock result during migration.
bool locked
# mask_source mirrors the current JSON key and is "NONE" when no mask is locked.
string mask_source

# ROI is [x1, y1, x2, y2] in image pixels and is valid only when has_tool_roi is true.
bool has_tool_roi
int32[4] tool_roi

bool has_reason
string reason
string payload_json
