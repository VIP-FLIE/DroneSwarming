# Repository coding rules

These rules apply to application code, tests, launch files, ROS interfaces, and build configuration. They apply to human-written and LLM-generated changes.

## Branch workflow and release approval

- Use `dev` as the shared integration branch. Keep `main` as the latest fully validated release.
- Create each feature or fix branch from the current `dev` branch.
- Push changes to the feature or fix branch, then open a pull request targeting `dev`.
- Do not push directly to `main`. Update `main` only through a release pull request from `dev`.
- Require approval from at least two team members before merging a release pull request into `main`.
- Keep `main` buildable, runnable, and ready for project demonstrations. A release candidate must pass the release checks in the [Repository Rules](docs/REPOSITORY_RULES.md), including the documented SITL mission without a vehicle crash.

## Standards are acceptance criteria

- Changes that do not follow these rules will not be accepted or merged.
- Apply the same requirements to human-written and LLM-generated changes.
- A successful build or passing generated tests does not waive a requirement in this document.
- Read the repository's [coding-standards skill](skills/coding-standards/SKILL.md) before changing code and the [docs-writing skill](skills/docs-writing/SKILL.md) before writing or editing documentation.

## Priorities

Apply these priorities in order:

1. Correctness, including edge cases and failure paths.
2. Safety, including input validation, resource ownership, and flight-control constraints.
3. Efficiency, using appropriate data structures and algorithms without unmeasured micro-optimization.
4. Minimality, with the least code and configuration that fully meets the requirement.

Minimality does not permit skipping validation, error handling, or required tests. Do not add unrequested features, options, dependencies, abstractions, or configuration. Add an abstraction only when it clarifies ownership or removes real duplication. Reuse project code and established APIs before adding new layers.

## Types and declarations

- Declare every function parameter and return type.
- Use the most specific type the language supports.
- Declare variables, fields, and constants with explicit types when their types are not obvious from the initializer.
- In C++, spell out local variable types. Do not use `auto` to shorten declarations or avoid repeating an obvious type.
- Use C++ `auto` only when the language requires type deduction or the type has no practical source-level name, such as a closure type. Keep each such use narrow and make the resulting type clear from nearby code.
- In Python, annotate every function parameter and return type. Annotate local variables when their types are not apparent from the assignment.
- Avoid `any`, `Object`, `void*`, unchecked casts, and other type-system escape hatches. If an API requires one, keep it at the boundary and document the reason.
- Use fixed-width integer types when a value has a protocol-defined range or width.
- Validate a value before narrowing or converting it to another numeric type.

## Naming and code structure

- Use full names that describe each variable, function, class, method, and module.
- Use single-letter names only for conventional short-lived loop indices and coordinate components.
- Name booleans so conditions read clearly, such as `is_valid`, `has_position`, or `should_land`.
- Use American English spelling in documentation and comments. Use `meter` and `meters` for distance units.
- Keep each function focused on one operation and name it for that operation.
- Replace unexplained numeric and string literals with named constants.
- Put units in physical quantity names, such as `_m`, `_hz`, and `_sec`.
- Remove dead code, unused imports, and unused variables.
- Follow existing formatting, build, test, and package conventions.

## Comments and documentation

- Document every production function and method for LSP hover. State its purpose or contract without repeating its name or signature.
- Keep function comments to three source lines or fewer. Exceed that limit only when a safety constraint or non-obvious invariant requires more explanation.
- Place C++ `///` comments immediately above declarations and Python docstrings as the first function statement. Put comments on header declarations instead of definitions when both exist.
- Write additional comments for constraints, invariants, non-obvious decisions, and workarounds.
- Do not use comments to repeat what the code already states.
- Update package documentation when changing a package's responsibility, public interface, runtime behavior, or required setup.
- Update the root architecture documentation when changing package boundaries, application coordinate conventions, PX4 integration, or flight-safety defaults.
- Keep ROS interface definitions in their source `.msg` and `.action` files. Do not edit generated build or install outputs.

## Safety and error handling

- Validate ROS parameters, action goals, and external telemetry at their input boundaries.
- Reject values outside documented ranges and reject non-finite numeric values used for positions, rates, distances, or timeouts.
- Handle every failure path. Do not swallow errors or report success after a failed operation.
- Translate errors into the owning ROS action result, state, or diagnostic at the relevant boundary.
- Use RAII and standard ownership types. Do not use owning raw pointers or manual `new` and `delete`.
- Use shared ownership only when multiple components share an object's lifetime. Prefer unique ownership otherwise.
- Keep cancellation behavior explicit for long-running ROS actions. Cancel active child actions and wait for their terminal results when the parent operation depends on them.
- Preserve a valid position hold and setpoint stream when cancelling vehicle navigation.
- Do not bypass PX4 preflight checks or failsafes.
- Keep `enable_flight_commands` disabled by default. Enable it only for a controlled simulation or hardware session after checking telemetry, system ID, topic prefix, and the operator's recovery path.

## ROS 2 and PX4 boundaries

- Keep application position contracts in `local_enu`, measured in meters. X is east, Y is north, and Z is up.
- Keep PX4-specific message types and DDS topic handling inside `vehicle_control`.
- Keep PX4-to-application frame conversion inside the `vehicle_control` PX4 adapter. Convert application `(east, north, up)` positions to PX4 `(north, east, down)` at that boundary.
- Keep `vehicle_control` as the only application package that depends on `px4_msgs`.
- Use application ROS actions for takeoff, navigation, landing, and mission execution. Keep higher-level mission logic independent of PX4 message types and topics.
- Derive every mission waypoint offset from the single position captured at mission start. Do not change this behavior without updating the mission interface documentation and tests.
- Keep `drone_bringup` responsible for launching application nodes and configuring the application namespace and PX4 topic prefix. It does not start PX4, Gazebo, a DDS agent, or a mission.
- Keep PX4 responsible for low-level flight control, preflight checks, and failsafes.
- Keep the supported ROS 2 Jazzy and PX4/`px4_msgs` v1.17.0 versions aligned across the Docker image, dependencies, and documentation. Version changes require an explicit compatibility update.

## Dependencies and development environment

- Use the project Docker environment for SITL and Gazebo workflows. Use [HITL-PI-SETUP.md](docs/setup/HITL-PI-SETUP.md) for native 64-bit Raspberry Pi builds on Ubuntu 24.04.
- Keep the Ubuntu version, ROS distribution, PX4 release, `px4_msgs` release, XRCE-DDS Agent release, and Ruff version aligned with `.env` and the pinned Docker base image.
- Build the same repository commit in Docker and on the Raspberry Pi. Do not add hardware-specific application behavior or interface definitions.
- Add a dependency only when existing project libraries and platform APIs do not meet the requirement.
- Keep Docker build dependencies pinned or otherwise reproducible through the existing Docker configuration.
- Use the documented editor setup for language servers so they resolve headers and compilation commands from the container.

## Tests and verification

- Add or update focused tests for behavior changes and deterministic calculations.
- Run the affected package tests in the project container.
- Run `colcon test-result --verbose` after `colcon test` and resolve reported failures.
- Build project-owned C++ targets and tests with the compiler diagnostics in `src/cmake/ProjectWarnings.cmake`. Treat warnings as errors and compile without GNU language extensions.
- Apply the strict warning profile to project targets only. Keep ROS and other dependency headers as system includes, and do not apply the profile to ROS-generated interface code.
- Run the `drone_bringup` package tests to check launch files with Ruff's full stable rule set and formatter. The project Docker image pins Ruff to version `0.17.1`.
- Validate flight-control changes in SITL before enabling flight commands in a live session.
- Record checks that were not run. Do not report an unrun build, test, or simulation as successful.

## Human and LLM contributions

- Apply the same rules to generated code and hand-written code.
- Review each generated change for correctness, explicit types, input validation, ownership, ROS/PX4 boundaries, and required tests.
- Be able to explain each abstraction, cast, numeric conversion, safety decision, and exception to the explicit-type rule.
- Do not accept generated code solely because it compiles or its generated tests pass.
