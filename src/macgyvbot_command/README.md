# macgyvbot_command

Command input package.

## Scope

- STT
- TTS
- local parser
- LLM fallback
- command validation
- `/tool_command` and `/command_feedback` publication

## Current State

This package now owns the command input node implementation, command parser,
and STT/TTS helpers.  Operator-facing GUI code lives in `macgyvbot_ui` and
communicates with this package through ROS topics.
