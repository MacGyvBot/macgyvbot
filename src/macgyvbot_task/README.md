# macgyvbot_task

Task orchestration package.

## Planned Scope

- command routing
- pick workflow
- return workflow
- handoff workflow
- task status publishing
- Python domain models used by workflows

## Current State

This package now owns the main task node, pick/return application workflows,
and task orchestration. It imports `macgyvbot_perception` and
`macgyvbot_manipulation` for perception/control adapters, with shared constants
and dataclasses coming from `macgyvbot_config` and `macgyvbot_domain`.
