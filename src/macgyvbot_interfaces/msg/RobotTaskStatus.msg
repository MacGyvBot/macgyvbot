# Robot task progress and UI status.
# task is optional and should be empty when the JSON payload does not publish it.
# command_json preserves the originating command payload during migration.
# payload_json preserves the complete existing JSON status shape during migration.
string status
string task
string tool_name
string action
string message
string reason
string command_json
string payload_json
