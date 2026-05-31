# Status of the currently locked tool mask used by hand grasp detection.
bool locked
# mask_source is "NONE" when no mask is locked.
string mask_source

# ROI is [x1, y1, x2, y2] in image pixels and is valid only when has_tool_roi is true.
bool has_tool_roi
int32[4] tool_roi

bool has_reason
string reason
