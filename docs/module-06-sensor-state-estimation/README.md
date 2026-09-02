# Module 6 — Sensor & State Estimation

## 1. Architectural Role

Module 6 is the state estimation and sensor fusion engine of TCAS.

Trains operate with wheel tachometers and odometers that suffer from measurement noise and accumulated drift caused by wheel slip and slide. Module 6 implements a discrete-time 1D Kalman Filter fusing continuous, drifting odometry with discrete physical track balise / transponder position fixes.

```text
Raw Odometry (noisy pos, vel, drift) + Track Balises (absolute anchor)
                                │
                                ▼
                       StateEstimator
                  (1D Discrete Kalman Filter)
             ├── Newtonian Kinematic Prediction (F, Q)
             ├── Innovation & Measurement Update (H, R)
             ├── Statistical Outlier Gating (3.5-sigma)
             └── Health & Degradation Assessment
                                │
                                ▼
                         EstimatedState
       (position, velocity, acceleration, uncertainty, health)
```

---

## 2. Public API

### `tcas::sensor::StateEstimator`

| Method | Description |
|---|---|
| `predict(dt, timestamp)` | Advances estimated state using kinematic transition matrix $F$ and covariance $Q$ |
| `updateOdometry(measurement)` | Fuses tachometer/odometer measurement with statistical outlier gating |
| `updateBalise(balise)` | Fuses discrete absolute track balise transponder anchor, resetting position uncertainty |
| `estimatedState()` | Returns `EstimatedState` structure with current estimates and $\sigma$ uncertainties |
| `isDegraded()` | Returns `true` if sensor failures or persistent measurement outliers are detected |
| `reset(pos, vel)` | Resets estimator state and initial covariance |

### `tcas::sensor::Odometer`

| Method | Description |
|---|---|
| `measure(pos, vel, acc, dt, tick)` | Generates sensor measurement with wheel slip drift accumulation |
| `calibrate(exactPosition)` | Calibrates odometer against an absolute balise fix, clearing accumulated drift |
| `setFaulty(faulty)` | Simulates hardware fault injection |
| `accumulatedDrift()` | Returns total accumulated drift error in metres |
