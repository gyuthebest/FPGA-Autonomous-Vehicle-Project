# PL Risk / Reliability Verification Coverage

Date: 2026-08-14

## Regression reuse policy

Previously passed live cases were not rerun. RTL regressions were rerun only
after the affected RTL changed. The final full regression record is
`sentinel_timeout_regression.out.log`, with **88 PASS / 0 FAIL**. It covers
preprocessing, range/jump/stuck/timeout/noise/consistency mechanisms, masks,
risk classifiers and control, AXI frame commit/packing, timeout recovery and
the no-target-sentinel timeout case.

The new gap-only regression is `tb_risk_reliability_matrix.sv`. Result:
**115 PASS / 0 FAIL**.

Combined current RTL coverage: **203 checks passed / 0 failed**. In addition,
the captured live AXI stream was replayed for **221 samples / 0 protocol or
sequence failures**.

## Live CARLA -> FPGA board -> CARLA verification

The same Control Panel state used by the operator was driven while Town04 was
running, sent over UDP to the physical FPGA board, and the returned PL risk,
reliability and vehicle-control words were checked for five consecutive final
samples.  The screen was captured for every scenario so that the applied CARLA
condition and resulting dashboard/control state can be reviewed together.

Aggregate result: **74 unique live scenarios passed / 0 failed**.

- Initial run: `CARLA_FPGA_PROJECT/logs/live_scenarios/20260814_001619`
  (20 cases passed; only the seven failed cases were scheduled again).
- WET correction evidence: `CARLA_FPGA_PROJECT/logs/live_scenarios/20260814_002342/01_road_wet.png`.
- Five corrected cases: `CARLA_FPGA_PROJECT/logs/live_scenarios/20260814_002920`
  (impact SEVERE/EXTREME, DIM visibility, yaw DANGER, lateral DANGER).
- Final 20% rough-road case: `CARLA_FPGA_PROJECT/logs/live_scenarios/20260814_003249`
  (**1 pass / 0 fail**). The raw ROUGH measurement makes acceleration Z
  DEGRADED against the dynamics reference, so the implemented reliability-to-
  risk rule correctly raises the effective PL road-impact state to SEVERE.

The 20 scenarios that had already passed were not repeated.  Corrections were
limited to the failed scenarios: safe LSB variation prevents an artificial
constant-value stuck fault, independent cases heal/reset diagnostic history,
and road-impact tests establish a visible >30 km/h physical precondition before
the bump is activated.

### Complete Sensor Fault control coverage

All 47 additional, semantically distinct Sensor Fault button paths were driven
through the running CARLA application and physical FPGA. Five sensor cases that
were already included in the 27 risk/reliability integration scenarios were
excluded from this 47-case set, and global timeout was tested once because all
per-sensor timeout buttons intentionally exercise the same PL transport path.

- Initial sensor matrix: `CARLA_FPGA_PROJECT/logs/live_scenarios/20260814_003812`
  (**27 pass**, 20 failures retained for diagnosis).
- Failed-only rerun: `CARLA_FPGA_PROJECT/logs/live_scenarios/20260814_004721`
  (**15 newly passed**, five remained).
- Stuck-trigger rerun: `CARLA_FPGA_PROJECT/logs/live_scenarios/20260814_011800`
  (**2 newly passed**, three remained).
- Gyroscope X/Y final run: `CARLA_FPGA_PROJECT/logs/live_scenarios/20260814_015318`
  (**2 pass / 0 fail**).
- Global timeout final run: `CARLA_FPGA_PROJECT/logs/live_scenarios/20260814_021129`
  (**1 pass / 0 fail**).

Aggregate sensor-button result: **47 / 47 passed**. Together with the first 27
scenarios, this gives **74 / 74 unique CARLA -> board PL -> CARLA cases**.

The live runs exposed and corrected three RTL integration defects:

- Distance stuck used an unreachable predictor-residual trigger; it now uses
  the independent nonzero closing-speed trigger.
- Acceleration/gyroscope stuck triggers used a second difference that repeatedly
  froze the debounce; they now use the intended direct reference delta.
- A confirmed transport timeout healed before PS could observe it, and the
  distance no-target override also erased it. The timeout is held across early
  recovery samples, and the no-target sentinel now masks only plausibility
  diagnostics while preserving the independent transport timeout.

One redundant, non-listening CARLA process was also removed during diagnosis.
It was reducing the live sample rate enough to repeatedly arm recovery masks;
the active Town04 server was left untouched.

## Reliability-to-risk dependency matrix

| Effective risk | Reliability source | DEGRADED behavior | INVALID behavior | TD group |
|---|---|---|---|---|
| Collision | Distance OR closing speed | Raise one tier, capped below maximum | Last valid + one tier, minimum DANGER | Yes |
| Road surface | Temperature OR humidity | Raise one tier | Last valid + one tier, minimum ICE | No |
| Road impact | Acceleration Z | Raise one tier | Last valid + one tier, minimum SEVERE | No |
| Light visibility | Illuminance | Raise one tier | Last valid + one tier, minimum DARK | No |
| Weather visibility | Simulation weather | No sensor-reliability dependency | No sensor-reliability dependency | No |
| Roll stability | Angular rate X | No extra binary tier exists | Force DANGER | Yes |
| Yaw stability | Angular rate Z | Raise SAFE to CAUTION | Last valid + one tier, minimum CAUTION | Yes |
| Lateral stability | Acceleration Y | Raise SAFE to CAUTION | Last valid + one tier, minimum CAUTION | Yes |
| Pitch diagnostic | Angular rate Y | Warning only | HUD warning | No |
| Longitudinal diagnostic | Acceleration X | Warning only | HUD warning | No |

## Newly verified gaps

- Every intermediate collision, road-surface, road-impact, light-visibility,
  weather, yaw and lateral control tier.
- NORMAL and DEGRADED mapping for every raw tier in every reliability-dependent
  risk group.
- INVALID conservative floor and last-valid-risk retention for each group.
- Weather independence from sensor reliability.
- Accelerator, brake, steering, speed-limit, headlight and hazard outputs.
- Speed-dependent collision braking branches.
- HUD-only invalid groups versus TD-triggering invalid groups.
- Multi-risk minimum/maximum/OR arbitration.
- Simultaneous risk conditions.

## CARLA scenario alignment

- Road surface selections continuously produce exact WET/ICE/BLACK ICE PL
  temperature/humidity classes.
- Visibility slider produces DIM/DARK/VERY DARK illuminance classes.
- Physical road impact starts a sensor response held for at least 2 seconds.
- Roll/yaw/lateral disturbances are applied continuously; each direction is
  held for at least 3 seconds before reversal.
- Yaw and lateral slider halves select CAUTION and DANGER tiers.
- Sensor-fault buttons remain active until toggled off, satisfying the PL
  sample-count debounces.
- A latched PL transition demand or MRM keeps FPGA control authority even after
  the initiating UI test button is released.

## Safety review finding (PL unchanged)

`risk_control.sv` currently sets `final_brake = 0` whenever road-surface risk is
ICE/BLACK ICE or lateral risk is DANGER. This also overrides a simultaneous
collision EMERGENCY request for brake 10. The gap regression proves that the
implemented result is accelerator 0, brake 0, hazard 1. This is internally
consistent with the current RTL but is a safety-policy conflict. It should be
resolved by the project team before changing RTL because the desired blended
braking policy on low-friction surfaces is a system-level safety requirement.
