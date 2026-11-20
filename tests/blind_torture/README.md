# Blind headless node torture tests

The [initial findings](FINDINGS.md) record observed failures, test assumptions, and validation limits.

This suite invokes installed nodes through ROS 2 messages, services, and actions. Test expectations come from the package README files, architecture documentation, `CONTRIBUTING.md`, and public `.msg`, `.srv`, and `.action` contracts. The author did not read node implementations, existing tests, or application libraries. The suite imports generated ROS interfaces and ROS client libraries. It does not import planner, executor, control, or geometry implementation modules.

`swarm_coordination` has no runtime behavior. `flight_test` requires SITL and is outside this synthetic-peer suite. `drone_bringup` is launch composition. The three tested runtime nodes are `mission_planner`, `mission_execution`, and `vehicle_control`.

## Execution

Use a dedicated project container with networking disabled and a private IPC namespace. The project image contains ROS 2 Jazzy and the PX4 message overlay. Build the current checkout before testing. From the host repository root:

```bash
docker run --rm --init --network none --ipc private \
  --mount "type=bind,src=$PWD,dst=/workspace" \
  --entrypoint bash drone-swarming:px4-v1.17.0 -lc '
    source /opt/ros/jazzy/setup.bash
    source /opt/px4_ros2/install/setup.bash
    cd /workspace
    colcon build --symlink-install --packages-up-to mission_execution &&
    source install/setup.bash &&
    python3 tests/blind_torture/run.py
  '
```

The container requires no Gazebo, PX4 executable, DDS agent, display, GPU, or vehicle. Synthetic PX4 messages use `/blind_px4/fmu`. Enabled vehicle-control scenarios use a loopback-only MAVLink listener. Flight-control outputs have no connected autopilot.

Inside a sourced, isolated project container:

```bash
python3 tests/blind_torture/run.py --list
python3 tests/blind_torture/run.py --seed 42 --fuzz-count 512
python3 tests/blind_torture/run.py \
  --case test_executor.ExecutorTorture.test_landing_preemption_drains_child_and_rejects_duplicates \
  --seed 42 --artifact-dir /workspace/run-logs/blind-replay-42
```

`--case` accepts an exact listed name and can repeat. The artifact directory must not exist. Replay uses the recorded case, seed, and fuzz count. DDS scheduling can vary between replays. Domain 213 and namespace `/blind` are reserved for each sequential worker. Run independent invocations in separate containers.

## Assertions

| Boundary | Adversaries and observable assertions |
| --- | --- |
| Planner JSON | Truncation, duplicate keys, excessive depth, UTF-8 byte limits, invalid schemas, booleans in numeric fields, NaN, infinities, invalid IDs, duplicate assignments, malformed unselected assignments, waypoint and polygon limits, invalid circles. Rejected requests require diagnostics. |
| Planner geometry | Seeded valid paths use an independent small-distance geographic oracle. Body rotation, exact GPS preservation, and vehicle assignment selection have explicit assertions. |
| Planner load | Seeded concurrent invalid requests must fail within deadlines. A valid canary must succeed after malformed input and floods. Trap action servers assert no flight side effects. |
| Executor admission | Malformed action goals, invalid telemetry, missing heading, stale streams, mixed waypoint frames, and goal floods cannot dispatch invalid or concurrent children. A valid navigation canary checks recovery. |
| Executor peers | Rejection, failure, unavailable servers, late acceptance, unconfirmed child termination, malformed planner responses, late planning responses, and planning timeouts must produce unsuccessful bounded outcomes. |
| Cancellation | Parent cancellation waits for child termination. Child cancellation failure preserves failure text. Explicit landing drains the active child, rejects duplicate landing and landing cancellation, and dispatches after the child terminal event. Invalid landing cannot interrupt. |
| Vehicle control | Default-disabled and telemetry-free actions cannot produce PX4 commands or setpoints. Synthetic telemetry checks ENU/NED conversion, finite validity flags, staleness, poisoned geographic references, malformed local goals, and invalid uploaded routes. |
| Integration | Real planner and executor nodes must translate and dispatch one uploaded geographic route to a synthetic vehicle backend. |

Every scenario runs in a fresh process group with a fresh node process. The default watchdog is 90 seconds. The aggregate worker budget is 1,536 MiB resident memory, 512 threads, and 64 MiB of artifacts. These are harness resource budgets. They are not flight performance requirements. A timeout, crash, budget violation, failed assertion, or missing dependency fails the scenario. The runner returns nonzero if any scenario fails and continues to the remaining scenarios.

## Artifacts and limits

Each case records wire requests, responses, child dispatch events, PX4 outputs, worker output, node logs, duration, peak sampled memory, and peak sampled thread count. The run records a manifest with harness file hashes, cumulative JSON results, and JUnit XML. Non-finite floating values appear as `NaN` or `Infinity` in JSONL wire traces using Python's JSON extension. Python can read these traces. Run artifacts default to the ignored `run-logs/` directory.

Typed DDS messages cannot carry arbitrary field types or invalid serialized bytes through generated ROS clients. The suite tests malformed values representable by those clients. It does not fuzz the DDS wire protocol, simulate vehicle dynamics, verify physical flight, or replace SITL validation. The synthetic child does not establish real PX4 hold. Cancellation tests verify executor ordering and reported child outcomes. Node defects remain failures; the suite does not modify application behavior or weaken assertions to accept observed failures.

The documented orbit bounds and standalone timeout ceiling define executor limit probes. These probes require rejection before child dispatch. The 121-meter standalone takeoff probe extrapolates the documented 120-meter mission envelope. A standalone takeoff ceiling and executor enforcement of vehicle limits are harness assumptions requiring contract review. An empty public waypoint list has no documented minimum length; its test requires a bounded, consistent outcome. Individual replies have a 5-second harness deadline. Planner floods have one shared 10-second deadline. These are test budgets, not additional ROS protocol guarantees. Flood tests retain the default ROS service QoS and report missing replies as liveness failures. They do not attribute message loss to a particular ROS layer.

The runner samples resource use every 100 milliseconds. It can miss shorter spikes. It does not claim leak freedom or exhaust all interleavings. Increase the fuzz count and run several seeds for additional generated input coverage.
