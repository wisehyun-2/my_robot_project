# AKE motor check of the user's robot (`python -m wbr.bench.ake_sizing`)

| motor | peak / rated [N m] | no-load [rpm] (rad/s) | gear | mass [kg] | reflected inertia [kg m^2] | backdrive [N m] |
|---|---|---|---|---|---|---|
| AKE50-8 KV110 | 7.5 / 2.5 | 320 (33.5) | 8:1 | 0.19 | 9.84e-04 | 0.09 |
| AKE60-8 KV80 | 12.5 / 5.0 | 240 (25.1) | 8:1 | 0.26 | 4.14e-03 | 0.12 |

## Per wheel diameter

| case | hip / knee / wheel motor | wheel [mm] | seed | V | mass [kg] | not recovered | wheel peak [N m] (scenario) | wheel RMS max | wheel sat % max | wheel speed max [rad/s] | v at 1.0 m/s cmd | hip peak (scenario) | hip RMS max | knee peak (scenario) | knee static max | knee RMS max | tire off the floor max [ms] (scenario) | slip max [m/s] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| payload5 | AKE60-8 / AKE60-8 / AKE60-8 | 202 | 0 | 24.0 | 15.571 | none | 12.50 (tilt_25) | 2.07 | 0.2 | 27.3 | 1.04 | 12.50 (push_1.5) | 2.62 | 11.13 (push_1.5) | 7.28 | 7.28 | 42 (slope_15_down_0.5) | 0.47 |
| robot | AKE60-8 / AKE60-8 / AKE60-8 | 202 | 0 | 24.0 | 10.571 | none | 7.41 (tilt_25) | 1.44 | 0.8 | 25.1 | 1.05 | 11.24 (push_1.5) | 1.61 | 8.19 (push_1.5) | 4.14 | 4.15 | 21 (push_1.5) | 0.02 |
