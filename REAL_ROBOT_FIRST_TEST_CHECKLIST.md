# WBC Real Robot First-Test Checklist

## Goal
Validate the updated state flow:
- Entry path: `PASSIVE -> FIXEDSTAND -> MJAMP -> SOCCER`
- Exit path: `SOCCER -> MJAMP` via `R2_B`
- Safety fallback: `L2_B -> PASSIVE`

## 1) Safety And Roles
- Confirm hardware emergency stop is available and tested.
- Assign roles: one operator (commands), one safety spotter (hands near stop).
- Clear the test area (at least 2m around robot).
- Start in low-risk mode (no high-speed switch in MJAMP during first run).

## 2) Communication And Topics
- Confirm network interface and DDS are healthy.
- Verify these topics are continuously updated:
  - `detectionresults`
  - `rt/locationresults`
  - `rt/soccer/ball_pos`
  - `rt/soccer/target_pos`
- If perception is stale, stop and fix comms before soccer behavior testing.

## 3) State-Flow Validation (No Ball)
- `PASSIVE -> FIXEDSTAND`: robot rises smoothly, no abrupt torque spikes.
- `FIXEDSTAND -> MJAMP` (`R2_A`): robot stabilizes without external support.
- `MJAMP -> SOCCER` (`R1_UP`): transition is smooth and posture remains stable.
- `SOCCER -> MJAMP` (`R2_B`): can be repeated multiple times without instability.

## 4) Light Closed-Loop Validation (With Ball)
- Run `SOCCER` for 30-60 seconds.
- Confirm ball and target updates look reasonable and continuous.
- Confirm operator can always:
  - return to `MJAMP` by `R2_B`
  - return to `PASSIVE` by `L2_B`

## 5) Abnormal Case Handling
- Immediate `L2_B` to `PASSIVE` if any shaking, drift, or perception dropout appears.
- Log each anomaly with:
  - command sequence
  - state transitions and timestamps
  - rough pre/post behavior notes (10s window is enough for first pass)

## 6) Acceptance Criteria
- Build passes and robot runs with no state-machine branch errors.
- Complete at least 5 bench cycles of `MJAMP <-> SOCCER`.
- On real robot, complete at least 3 full cycles:
  - `FIXEDSTAND -> MJAMP -> SOCCER -> MJAMP`
- After exiting to `MJAMP`, robot can re-enter `SOCCER` again.
