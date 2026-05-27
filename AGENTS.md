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
- The root `README.md` is the entry point for humans and agents. Update it
  whenever package roles, launch paths, setup steps, or common workflows change.
- Package-level details can live in `EXPLAIN.md` or `docs/`, but the root
  `README.md` must stay accurate enough to route a new contributor.

## Source Package Responsibilities

Each folder directly under `src/` is a ROS 2 package. Add new code to the
package that owns the behavior instead of creating cross-package shortcuts.

- `src/macgyvbot_bringup`: launch composition and startup wiring. Keep runtime
  policy, parsing, perception, and manipulation logic out of launch files.
- `src/macgyvbot_task`: task orchestration, command routing, pick/return flows,
  task status, pause/resume/exit, and cleanup. Keep ROS node files thin and put
  reusable flow logic under `macgyvbot_task/application/`.
- `src/macgyvbot_command`: command input before it becomes a robot task,
  including STT, TTS, command parsing, vocabulary, and command feedback.
- `src/macgyvbot_ui`: operator-facing presentation. Communicate through ROS
  topics or stable adapters, not through command/perception/manipulation
  internals.
- `src/macgyvbot_perception`: YOLO, VLM/API grasp point selection, depth
  projection, pick target resolution, and human hand/tool grasp perception.
  Runtime code must not depend on `data/` or `train/` scripts.
- `src/macgyvbot_manipulation`: MoveIt, gripper, force sensing, robot pose,
  safe workspace, grasp verification, tool drop monitoring, and handoff targets.
  Keep hardware assumptions and motion parameters localized and documented.
- `src/macgyvbot_config`: shared runtime constants and ROS topic names. Prefer
  `macgyvbot_config.topics` for cross-package topic constants.
- `src/macgyvbot_domain`: shared in-process Python dataclasses and domain
  models. Keep it free of ROS node lifecycle, UI, hardware, model runtime, and
  network dependencies.
- `src/macgyvbot_interfaces`: typed ROS messages and migration away from ad hoc
  JSON string payloads. Message changes are API changes.
- `src/macgyvbot_resources`: calibration files, model weight download scripts,
  asset placeholders, and resource package metadata.

## Source Connection Rules

- `macgyvbot_bringup` connects packages at launch time.
- `macgyvbot_command` converts user input into command topics or feedback.
- `macgyvbot_task` consumes commands, coordinates perception and manipulation,
  and publishes task status.
- `macgyvbot_perception` publishes or returns perception results; it should not
  command robot motion directly.
- `macgyvbot_manipulation` executes robot-facing operations requested by task
  orchestration; it should not parse user commands or own UI state.
- `macgyvbot_ui` presents state and sends operator intent through stable ROS
  contracts; it should not reach into internals of command, perception, or
  manipulation packages.
- `macgyvbot_config`, `macgyvbot_domain`, `macgyvbot_interfaces`, and
  `macgyvbot_resources` are shared support packages. They should not import
  application packages such as task, command, UI, perception, or manipulation.

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
- Motion safety, unintended movement, IK solution selection, joint flip
  prevention, and trajectory sanity checking belong to issue #89 unless the
  active issue says otherwise.

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

- Update the root `README.md` for any change that affects repository structure,
  package responsibilities, setup, build, launch, common workflows, or the
  location of important documentation.
- When adding, moving, or renaming files under `src/`, check whether the root
  `README.md` still routes contributors to the correct package and workflow.
- Update `EXPLAIN.md` for package roles, major flows, or file responsibility
  changes.
- Update `docs/architecture/topics.md` for topic contract changes.
- Keep long design notes in `docs/`, not in this guide.
