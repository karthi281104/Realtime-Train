# Module 8 — Predictive Position Engine

## 1. Architectural Role

Module 8 is the continuous multi-horizon kinematic prediction engine of the Real-Time Train Collision Avoidance System (TCAS).

Because heavy passenger and freight trains have high momentum and require substantial stopping distances, collision avoidance cannot be achieved using instantaneous location updates alone. The `PredictionEngine` computes the future spacetime coordinates of each train across multiple forward horizons ($T = \{5\text{s}, 10\text{s}, 20\text{s}, 30\text{s}, 60\text{s}\}$), projecting along the planned track topology while strictly enforcing speed limits and gradient physical constraints.

```text
               Train State & Route Navigation
                             │
                             ▼
                     PredictionEngine
           ├── Track Boundary & Distance Lookahead
           ├── Gradient-Induced Gravitational Acceleration
           ├── Downstream Speed Limit Lookahead Braking
           └── Time-Dependent Uncertainty Propagation
                             │
                             ▼
              Ordered std::vector<FutureState>
          [timestamp, trackId, position, velocity, uncertainty]
```

---

## 2. Mathematical Formulation

### 2.1 Forward Kinematic Projection
For a current state $(x_0, v_0, a_0)$ and projection horizon $\Delta t$:

$$v(t) = \min\left(v_{\text{track\_limit}}, \max\left(0, v_0 + a_{\text{eff}} \cdot \Delta t\right)\right)$$

$$x(t) = x_0 + v_0 \cdot \Delta t + \frac{1}{2} a_{\text{eff}} \cdot \Delta t^2$$

Where effective acceleration $a_{\text{eff}}$ incorporates the local track gradient:

$$a_{\text{eff}} = a_{\text{engine}} - g \cdot \sin(\theta) \approx a_{\text{engine}} - g \cdot \text{grade}$$

### 2.2 Boundary Time & Multi-Track Transitions
When the projected position $x(t) > L_{\text{track}}$, the exact transition timestamp $t_{\text{boundary}}$ is computed:

$$t_{\text{boundary}} = \frac{-v_0 + \sqrt{v_0^2 + 2 a_{\text{eff}} \cdot (L_{\text{track}} - x_0)}}{a_{\text{eff}}}$$

The remaining interval $\Delta t - t_{\text{boundary}}$ is recursively propagated into the subsequent track in the Dijkstra route.

### 2.3 Uncertainty Propagation
To account for odometry drift and Kalman filter variance over extended forecast horizons:

$$\sigma(t) = \sigma_0 + \alpha_{\text{drift}} \cdot t$$

---

## 3. Public API

### `tcas::prediction::PredictionEngine`

| Method | Description |
|---|---|
| `predictStandardHorizon(...)` | Computes future states at standard ETCS horizons: $5\text{s}, 10\text{s}, 20\text{s}, 30\text{s}, 60\text{s}$. |
| `predict(...)` | Computes future states for arbitrary user-specified horizon vectors. |
| `predictInto(outTrajectory, ...)` | **Zero-allocation API**: Reuses a pre-allocated trajectory buffer across periodic cycles. |

### `tcas::prediction::FutureState`

```cpp
struct FutureState
{
    TimeSeconds horizonTime{ 0.0 };
    TrackId trackId{ 0 };
    DistanceMeters position{ 0.0 };
    SpeedMetersPerSecond velocity{ 0.0 };
    AccelerationMetersPerSecondSquared acceleration{ 0.0 };
    DistanceMeters uncertainty{ 0.0 };
};
```

---

## 4. Verification & Testing

- **`tests/prediction/PredictionEngineTest.cpp`**:
  - `PredictsConstantVelocity`
  - `RespectsTrackSpeedLimits`
  - `CrossesTrackBoundariesAccurately`
  - `StopsAtEndOfPlannedRoute`
  - `PropagatesUncertaintyMonotonically`
  - `HandlesDecelerationToStop`
  - `BufferReuseWithPredictInto`
