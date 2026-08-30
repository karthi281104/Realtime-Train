# Module 4 — Physics & Braking

## 1. Architectural Role

Module 4 is the kinematics and braking dynamics engine of TCAS.

It provides deterministic mathematical functions for train motion simulation, gradient-corrected braking deceleration, driver reaction distance calculation, and safe stopping distance estimation.

```text
       Track (gradient) + Train (mass, v_max, a_brake, a_emerg)
                                  │
                                  ▼
                        KinematicsEngine
                                  │
      ┌───────────────────────────┼───────────────────────────┐
      ▼                           ▼                           ▼
Position / Velocity      Effective Deceleration      Safe Stopping Distance
Update Equations         with Track Gradient         (reaction + brake + margin)
```

---

## 2. Public API

### `tcas::physics::KinematicsEngine`

All methods are static pure functions with zero instance state:

| Method | Description |
|---|---|
| `updatePosition(x, v, a, dt)` | Position update: x = x0 + v0*dt + 0.5*a*dt^2 |
| `updateVelocity(v, a, dt, v_max)` | Velocity update: v = clamp(v0 + a*dt, 0, v_max) |
| `effectiveDeceleration(a_nom, grad)` | Gradient correction: a_eff = max(a_nom + g*grad, a_min) |
| `brakingDistance(v, a_eff)` | Braking distance: d = v^2 / (2 * a_eff) |
| `reactionDistance(v, t_react)` | Reaction distance: d_r = v * t_react |
| `safeDistance(v, a_nom, grad, t_r, margin)` | Safe distance: d_safe = d_r + d_brake + margin |
| `emergencyStoppingDistance(v, a_emerg, grad)` | Emergency stop: d_emerg = v^2 / (2 * a_emerg_eff) |
| `speedAfterDistance(v0, a, dist)` | Velocity after distance: v = sqrt(max(v0^2 + 2*a*d, 0)) |
