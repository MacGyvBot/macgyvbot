# MacGyvBot Agent Guide

This guide defines the working contract for contributors and coding agents.
Use it to keep refactoring scoped and aligned with the current ROS 2 package
architecture.

## Start From The Active Issue

- Before editing, identify the active issue or task.
- If the scope is unclear, ask for the GitHub issue or a concise task boundary.
- When an issue is provided, read it first and treat it as the source of scope.
- Do not make out-of-scope fixes opportunistically. Report them and suggest a
  follow-up issue instead.

## Current Architecture

- Runtime code lives under `src/` as ROS 2 packages.
- `macgyvbot_bringup`: launch composition.
- `macgyvbot_task`: task queue, command routing, pick/return orchestration,
  robot task status.
- `macgyvbot_command`: headless command pipeline, parser, STT, TTS, command
  feedback.
- `macgyvbot_ui`: operator-facing GUI and UI presenters.
- `macgyvbot_perception`: YOLO, VLM/API grasp point selection, depth projection,
  hand grasp perception.
- `macgyvbot_manipulation`: MoveIt adapter, gripper, force sensing, robot pose,
  safe workspace, grasp verification, handoff targeting.
- `macgyvbot_config`: shared runtime constants.
- `macgyvbot_domain`: shared in-process Python dataclasses.
- `macgyvbot_resources`: calibration and model asset ownership.
- `macgyvbot_interfaces`: typed ROS message migration target.

## Refactoring Boundaries

- Keep ROS node files focused on parameters, subscriptions, publishers, and
  delegation.
- Move policy, parsing, validation, selection, and calculations into testable
  helpers.
- Keep behavior changes separate from structural refactors unless the issue
  explicitly combines them.
- Preserve public interfaces unless changing them is the point of the issue.
- Respect package import direction:
  - `macgyvbot_command` must not import `macgyvbot_ui`.
  - `macgyvbot_ui` must not import command parser, STT, or TTS internals.
  - Shared constants belong in `macgyvbot_config`.
  - Shared Python models belong in `macgyvbot_domain`.

## Area Ownership

- Safety / Task Control: task queue boundaries, pause/resume/exit, cleanup,
  task status consistency.
- VLM / Perception: VLM runtime, prompts, response parsing, grid selection,
  SAM input preparation, depth refinement.
- Command / UI: package separation, topic contracts, GUI presentation. Do not
  change parser behavior unless requested.
- Manipulation / Return / Handoff: handoff target observation, return handoff
  policy, placement flow boundaries, gripper verification.
- Drawer: drawer open/close primitives and drawer-specific pick/return behavior.
- Motion safety, unintended movement, IK solution selection, joint flip
  prevention, and trajectory sanity checking belong to issue #89 unless the
  active issue says otherwise.

## Topic Contracts

- Treat ROS topics between packages as API contracts.
- Prefer topic constants from `macgyvbot_config.topics`.
- Keep current JSON-over-`std_msgs/String` payloads stable unless a dedicated
  migration issue changes them.
- Update `docs/architecture/topics.md` when topic ownership, direction, or
  payload shape changes.

## Robot Safety

- Do not change motion parameters, offsets, force thresholds, safe workspace
  limits, or gripper timings as part of an unrelated refactor.
- If motion behavior changes, document the reason and verification plan.
- Planning failure, missing depth, missing detections, unsupported frames, and
  interrupted tasks must be handled explicitly.

## Verification

Run focused checks for the touched area. Useful defaults:

```bash
python3 -m compileall -q src
python3 -m pytest -q src/macgyvbot_manipulation/test/test_handover_targeting.py
python3 -m pytest -q src/macgyvbot_manipulation/test/test_gripper_grasp.py
python3 -m pytest -q src/macgyvbot_perception/test/test_hand_grasp_ml_mask.py
python3 -m pytest -q src/macgyvbot_task/test
```

When ROS, MoveIt, camera, model, network, or robot hardware dependencies prevent
verification, state what was not run and what remains to verify.

## Documentation

- Update `README.md` for user-facing install, build, or launch changes.
- Update `EXPLAIN.md` for package roles, major flows, or file responsibility
  changes.
- Update `docs/architecture/topics.md` for topic contract changes.
- Keep long design notes in `docs/`, not in this guide.
