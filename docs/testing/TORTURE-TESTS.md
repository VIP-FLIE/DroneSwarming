# Flight torture tests

`scripts/torture-tests` runs 36 adversarial cases against the real `vehicle_control`, `mission_execution`, and `mission_planner` executables. A synthetic PX4 protocol peer supplies telemetry and command acknowledgments. Each case runs in a supervised process with its own ROS domain, namespace, PX4 topic prefix, parameters, random seed, and native ROS log directories.

The suite measures application behavior under communication faults, invalid estimates, ownership changes, command rejection, concurrency, and process failures. The peer provides protocol responses and optional instantaneous position feedback. It does not model aircraft dynamics, PX4's estimator, or PX4's failsafe implementation. A passing result does not establish wind tolerance, motor-failure tolerance, braking distance, or hardware readiness.

## Build and run

Build the current workspace before testing. The host wrapper uses the non-GUI `development` service and requires no simulator session or desktop display:

```bash
docker compose build development
docker compose run --rm --no-deps -T --entrypoint /bin/bash \
  -e BASH_ENV=/dev/null development --noprofile --norc -c \
  'source /opt/ros/jazzy/setup.bash && source /opt/px4_ros2/install/setup.bash && cd /workspace && colcon build --symlink-install'
./scripts/torture-tests
```

In an existing sourced project container:

```bash
./scripts/torture-tests
```

The wrapper requires the complete workspace, including mission execution and the planner. Its Python ROS dependencies are declared as vehicle-control test dependencies. The full adversarial suite is opt-in because existing defects are expected to fail its safety contracts. The three report-integrity unit tests also run through `colcon test --packages-select vehicle_control`.

List cases, rerun one fault, or repeat every case with deterministic seeds:

```bash
./scripts/torture-tests --list
./scripts/torture-tests --case test_position_dropout_aborts_navigation --seed 1731
./scripts/torture-tests --repeat 20 --seed 1731
```

`--case` accepts an exact name and can be repeated. Repetition zero uses the supplied seed; each later repetition increments it. `--artifact-dir` selects a new, nonexistent output directory. The default is `run-logs/torture/<UTC timestamp>-<suffix>/`. The command returns zero only when every case passes, one for contract/process/timeout failures, two for suite infrastructure errors, and 130 for interruption.

## Failure evidence and file logging

Application nodes use their native ROS external logging backend with `ROS_LOG_DIR` set separately for each node. Application loggers run at Debug; middleware loggers run at Info. Node stdout/stderr is discarded. The harness writes its own files directly. It does not derive diagnostics by capturing terminal streams.

| File or directory | Contents |
| --- | --- |
| `report.md` | Per-case verdict, seed, duration, and evidence links |
| `summary.json` | All completed results and counts, refreshed after every case |
| `junit.xml` | Contract failures, process errors, and watchdog timeouts with separate classifications |
| `run.json` | Full source commit, dirty status, platform, Python, seed, environment whitelist, and test-source hashes |
| `source.patch`, `test-source/` | Tracked-source diff and the archived Python tests executed by every child |
| `runner.log` | Suite/child bootstrap and supervision exceptions |
| `<case>/case.log` | Unittest assertions and complete Python failure tracebacks |
| `<case>/result.json` | Verdict, assertion/error detail, separate cleanup errors, recent events, stream counts, and peak process RSS |
| `<case>/events.jsonl` | Injected samples, dropped samples, ACKs, commands, setpoints, heartbeats, state, goals, feedback, results, assertions, faults, process events, and resource samples |
| `<case>/observer/` | Native ROS file log for the fault peer's phase/fault/assertion events |
| `<case>/{vehicle_control,mission_execution,mission_planner}/` | Native application ROS log files; sanitizer reports when enabled |
| `<case>/mission_planner/python-faults.log` | Uncaught Python exception tracebacks and fatal-signal diagnostics written directly to a file |
| `<case>/*-parameters.json` | Exact per-node ROS parameters supplied at startup |

Every event includes a sequence number, monotonic timestamp, wall timestamp, and elapsed seconds. ROS payloads retain all fields as YAML strings inside standard JSON, including NaN and infinity. A failed assertion records its measured and expected values before raising. Process-start events record executable arguments and binary SHA-256 hashes. Process RSS, CPU time, process state, and thread counts are sampled every 0.25 seconds.

The [Jazzy native logging backend](https://github.com/ros2/rcl_logging/blob/jazzy/rcl_logging_spdlog/src/rcl_logging_spdlog.cpp) flushes periodically at five seconds and on Error messages. Abrupt termination can lose recent native log messages. The independently flushed `events.jsonl` retains observed ROS traffic and process exit codes across those failures. An unsanitized C++ abort does not provide a C++ stack trace through this harness.

For a failed case, read `result.json`, then locate the last `fault`, `phase`, `check`, or `deadline_exceeded` event in `events.jsonl`. Correlate its wall timestamp with the native ROS files. For a process error, inspect the recorded exit code and the node's log directory. For a timeout, the parent retains the trailing complete events and kills the entire owned process group.

An unexpected nonzero application exit or a required SIGKILL during ordinary shutdown makes the case an error. Injected process deaths are recorded separately. A contract failure and a shutdown error can occur in the same case; `failures` and `cleanup_errors` retain both. A successful flight-contract assertion does not erase a shutdown error.

An example failure timeline query:

```bash
python3 - <<'PY'
import json
from pathlib import Path

case = Path('run-logs/torture/<run>/<case>')
for line in (case / 'events.jsonl').read_text().splitlines():
    event = json.loads(line)
    if event['kind'] in {'fault', 'phase', 'check', 'deadline_exceeded', 'action_result'}:
        print(json.dumps(event, indent=2))
PY
```

## Fault coverage and acceptance

| Case group | Injected conditions | Required behavior |
| --- | --- | --- |
| Healthy controls | Fresh state, accepted ACKs, synthetic destination feedback | Navigation and takeoff/landing assertions succeed before fault results are interpreted; teardown errors remain separate evidence |
| Passive startup | Disabled node; enabled restart beside an existing Offboard flight | No unrequested setpoints, heartbeat, or commands |
| Telemetry loss | Complete position loss; complete control-status loss | Active navigation reports failure within the defined reaction budget |
| Stale control | Position/status/land streams stop together | An expired control snapshot cannot sustain an indefinite healthy heartbeat |
| Replay/reordering | Frozen source timestamp; a five-second-old sample with a different position | Replayed data becomes invalid; an older sample cannot replace valid current state |
| Estimate corruption | NaN position; invalid/infinite yaw | Fail the affected operation and preserve finite outgoing position targets |
| Estimator resets | XY reset during navigation; Z reset during a trajectory | Invalidate targets from the earlier frame |
| External takeover | Manual mode; low-battery Land mode; Manual during Orbit recovery | Abort application motion without reclaiming control or claiming arrival |
| Parameter truthfulness | Runtime command-disable update | Successful updates take effect; immutable settings explain rejection |
| Numeric boundaries | `1e300` position; `3e38` seconds; excessive trajectory speed | Reject before publication or clock conversion |
| Route execution | Vehicle remains at a trajectory's start/final point and misses its intermediate point | Never claim successful route execution |
| Whole-mission validation | Later Orbit radius exceeds the controller limit | Reject before arming |
| PX4 command failures | Total ACK loss; wrong requester ACK; unrelated ACK bursts; explicit arming denial | Bound retries, retain the relevant ACK, propagate a stage-specific reason, and never bypass arming checks |
| Command concurrency | 24 simultaneous navigation requests; cancellation during takeoff prestream or racing landing | Admit one ordinary operation and serialize terminal landing |
| Owner/backend failure | Mission process killed; vehicle process killed or frozen with SIGSTOP | Bound recovery reporting, preserve unknown outcomes, and avoid orphaned healthy control |
| Planning failures | Missing planner; malformed, oversized, deeply nested, duplicate-key and non-finite JSON | No takeoff or arming side effect |
| Compound disturbance | Seeded position/status loss, delivery delay/jitter/reordering, position disturbance and velocity spikes | Never fabricate arrival or publish non-finite position targets |
| Action churn | Twenty navigation/cancel cycles | Every cycle terminates; process RSS growth stays below a coarse 32-MiB budget |

The issue references in each test's docstring link these contracts to [issues.md](../../issues.md). Tests use the existing public operation API except for rejection/passivity checks that intentionally exercise the vehicle-control boundary directly.

## Timing and resource bounds

The default telemetry freshness threshold is 0.3 seconds. Stream-loss and frame/mode reaction checks allow 1.2 seconds. These are scaled test settings, not newly certified flight limits. The normal navigation deadline is three seconds. Command ACK waits are 0.2 seconds and mode-transition waits are 0.5 seconds. Startup waits for complete telemetry and both application-state consumers before faults begin.

Backend-death and frozen-backend cases allow 19 seconds for a result because the current mission executor has a 15-second cancellation grace. Landing precedence is tested independently with a five-second protocol budget. A bounded failure with an unknown outcome is distinct from a successful landing. A frozen or dead backend cannot demonstrate physical recovery.

Each case has an independent 35-second wall-clock watchdog and a 50,000-event diagnostic limit. Cases run sequentially. The parent kills the owned process group after completion or timeout, including stopped children. Topics always use a UUID-generated `/torture_.../fmu` prefix; the suite accepts no vehicle-prefix override. Child ROS domains are 180 through 199 with localhost discovery. Seeds reproduce pseudo-random choices; operating-system scheduling remains variable.

The RSS budget detects gross resource growth. It does not establish absence of leaks. ROS result retention and allocator caches affect RSS. Use sanitizers for ownership and numeric defects.

## Sanitizer pass

Build an instrumented C++ overlay in the project container. Use separate AddressSanitizer and UndefinedBehaviorSanitizer passes. The cached GCC 13.3 runtime loses UBSan file output in a combined build, consistent with [GCC sanitizer report 94328](https://gcc.gnu.org/pipermail/gcc-bugs/2020-March/695568.html).

```bash
colcon build --symlink-install --build-base build-ubsan --install-base install-ubsan \
  --packages-up-to mission_execution mission_planner \
  --cmake-args \
  '-DCMAKE_CXX_FLAGS=-g -fsanitize=undefined,float-cast-overflow -fno-omit-frame-pointer -fno-sanitize-recover=all' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=undefined,float-cast-overflow'
source install-ubsan/setup.bash
./scripts/torture-tests --case test_unrepresentable_timeout_is_rejected \
  --case test_finite_overflow_position_is_rejected \
  --case test_repeated_cancel_cycles_remain_bounded
```

For the AddressSanitizer pass, use `build-asan`, `install-asan`, and `-fsanitize=address` in both flag arguments, then source `install-asan/setup.bash` and run the cases. Source the normal workspace in a new shell before selecting the other overlay.

The harness sets sanitizer `log_path` separately for each application process. Reports use `ubsan.<pid>` or `asan.<pid>` in the node's case directory. Dependency-origin reports require attribution before being classified as application defects. The native project compiler warning profile remains active.

## Recorded validation

Validation on November 13, 2026 used source commit `66c6bb80f905d2ad9a730566ef3749f6678f9fb4` with the local test changes. The cached image was `sar-drone:px4-v1.17.0`, ID `sha256:6244ed6774f545a48fdfc76fc0f3436bbc7e5eefd032fdd551e68b835a175115`, with ROS Jazzy, Python 3.12, and PX4/`px4_msgs` v1.17.0. This environment was not a rebuild of the current Dockerfile. Containers used no network, a read-only source mount, and separate build/output mounts.

- The [36-case retained report](../../run-logs/torture/verified-20261113-s1731/report.md), seed 1731, records 11 passing cases, nine contract-failure verdicts, sixteen error verdicts, and zero watchdog timeouts. Seventeen cases contain contract assertions that failed. Sixteen cases contain shutdown errors. Eight cases contain both; verdict counts classify those cases as errors.
- Both healthy controls completed their operation assertions. Both recorded planner shutdown errors. The traceback identifies the invalid ROS context race described in [ISS-40](../../issues.md#iss-40--p2--observed-defect-planner-signal-shutdown-can-exit-with-an-uncaught-ros-context-error). Fifteen nonempty direct Python exception files retain that traceback. Two cases also recorded vehicle-control SIGABRT during teardown; their precise C++ cause is unresolved. One case contains both application exit errors.
- The unrelated-ACK burst case recorded repeated transition ACK timeouts. This protocol test does not distinguish DDS depth-one message loss from replacement in the application's ACK cache. Its failure establishes the observed transition failure under that traffic.
- The 20-cycle action-churn case passed its completion and coarse memory-growth contracts. A separate three-repetition command-storm check passed after the test client was changed to drain its own DDS response queue while requests overlap.
- `colcon test --packages-select vehicle_control drone_bringup` and `colcon test-result --verbose` completed with 120 recorded test results, zero errors, zero failures, and zero skips in the retained build workspace. Bringup checks used pinned Ruff 0.17.1. The three report-integrity tests also passed directly on the host.
- The [initial combined-sanitizer run](../../run-logs/torture/sanitized-20261113-s1731/report.md) built both C++ applications with AddressSanitizer, UndefinedBehaviorSanitizer, and float-cast-overflow instrumentation. Healthy navigation and churn assertions completed; shutdown errors remained failures. UBSan file output was absent. A separate minimal GCC probe reproduced terminal-only UBSan output in the combined configuration. The separate-pass procedure above replaces that configuration.
- A [UBSan-only vehicle-control run](../../run-logs/torture/ubsan-20261113-s1731/report.md) completed healthy navigation assertions and reproduced the duration conversion defect through the real navigation action. The direct `vehicle_control/ubsan.<pid>` file records an out-of-range `3e+47` conversion in `std::chrono`, called by `deadline_after` and `execute_navigation`. A [final diagnostic rerun](../../run-logs/torture/ubsan-file-logging-20261113-s1731/report.md) verified that `result.json` indexes the sanitizer file and that the archived test hashes match the completed test source. Planner shutdown errors remain recorded separately.

The retained reports and node logs are ignored local artifacts. Copy the run directory to the team's evidence storage before deleting it. The archived Python source, recorded binary hashes, parameters, and source metadata identify the tested inputs.

## Physics and PX4 acceptance still required

The following conditions require real PX4/Gazebo execution or hardware validation. They are outside the synthetic peer's verdict and must remain separate acceptance evidence:

| Physical/system scenario | Evidence required |
| --- | --- |
| Calm air through the airframe's specified wind/gust limit; takeoff, turns, braking and landing | Measured position/velocity, stopping distance, actuator saturation, estimator quality, and landed confirmation |
| GPS loss, sensor freeze/bias, estimator recovery, and local-origin reset | PX4 estimator/failsafe state and bounded application response |
| Offboard link loss, RC/GCS loss, and companion CPU starvation | Actual configured PX4 recovery mode, reaction time, and continued operator authority |
| Battery warning/critical/emergency transitions | Actual PX4 configured action, voltage/current model, available flight time, and landing outcome |
| Motor/actuator failure, degraded thrust, and saturation | Supported failure injection, airframe-specific controllability, and actual PX4 response |
| Multi-vehicle congestion and vehicle-ID/prefix mistakes | Separation, exclusive ownership, observed DDS/network delay, and no cross-vehicle commands |

PX4's supported injection combinations depend on the simulator. Record unsupported/NACKed injections as unsupported rather than passed. Retain ULog, simulator configuration, effective failsafe parameters, and the application logs for every physical/SITL case. Use the pinned v1.17 [failsafe simulation guide](https://docs.px4.io/v1.17/en/simulation/failsafes) and [failure command reference](https://docs.px4.io/v1.17/en/modules/modules_command#failure). The existing [flight procedures](README.md) provide the controlled SITL workflow.

No finite fault suite establishes tolerance of every simultaneous failure. A quadrotor motor failure and loss of all attitude sensors can exceed the vehicle's controllability. Acceptance requires a defined operating envelope and a defined PX4 response for faults outside it.

Assumptions: each vehicle has one application authority; operator and PX4 takeover remain authoritative; the existing mission-start ENU contracts remain required. Unmet validation: the current Dockerfile rebuild, real PX4/Gazebo fault flights, hardware tests, airframe wind/thrust limits, multi-vehicle separation, thread-sanitizer execution, and exhaustive fault combinations. No coding-standard conflict was required. Application behavior remains unchanged; failing contracts require separate source fixes.
