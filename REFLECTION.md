# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model
The simple kinetic model is used in this project.

The state of the model is described X, Y coordinates, the Orientation angle Psi, and Velocity v.
```
 State: [x,y,ψ,v]
```

Actuator inputs allow us to control the vehicle state. Most cars have three actuators: the steering wheel, the throttle pedal and the brake pedal. For simplicity I'll consider the throttle and brake pedals as a singular actuator. This reduced actuators are described δ for steering angle and a for acceleration.
```
Actuators: [δ,a]
```

The update equations describe how the state changes over time based on the previous state and current actuator inputs.
```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
```

The model works by tracking two errors: Cross Tracking Error (CTE) and Orientation Error (Epsi), and by attempting to reduce these two errors as it follows a reference trajectory.
```
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

## Timestep Length and Elapsed Duration
N is the number of timesteps in the horizon. dt is how much time elapses between actuations. The prediction horizon is the duration over which future predictions are made. We’ll refer to this as T.

First, I set the value of T to 1 and try to adjust N and dt accordingly. I looked at quizz provided by udacity and set N to 25 and dt to 0.05. For tuning, I tested the model using the waypoints in the lake_track_waypoints.csv file and visualized the result using the C ++ matplotlib wrapper.

```
./mpc tuning
```

Finally, I set N to 10 and dt to 0.1 and confirmed that it works well in the simulator.

## Polynomial Fitting and MPC Preprocessing
All computations are performed in the vehicle coordinate system. So, all the points was first conveted to the vehicle coordinate system. The following equations were used.

```
c_ptsx[i] = cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
c_ptsy[i] = -sin(psi) * (ptsx[i] - px) + cos(psi) * (ptsy[i] - py);
```

In the vehicle coordinate system, the initial position of the car and heading direction are always zero. Thus the state of the car is as follows.

```
state << 0, 0, 0, v, cte, epsi;
```

## Model Predictive Control with Latency
The simulator has a 100ms latency to mimic real driving conditions. To handle this, I calculated the cross track and orientation error using the predicted state instead of the current state. The following equation is used to predict the new state using the current state of the car received from the simulator and the given latency.

```
px = px + latency * v * cos(psi);
py = py + latency * v * sin(psi);
psi = psi + v * delta / Lf * latency;
v = v + a * latency;
```