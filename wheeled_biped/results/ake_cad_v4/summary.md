# AKE motor check of the user's robot (`python -m wbr.bench.ake_sizing`)

| motor | peak / rated [N m] | no-load [rpm] (rad/s) | gear | mass [kg] | reflected inertia [kg m^2] | backdrive [N m] |
|---|---|---|---|---|---|---|
| AKE50-8 KV110 | 7.5 / 2.5 | 320 (33.5) | 8:1 | 0.19 | 9.84e-04 | 0.09 |
| AKE60-8 KV80 | 12.5 / 5.0 | 240 (25.1) | 8:1 | 0.26 | 4.14e-03 | 0.12 |

## Per wheel diameter

| case | hip / knee / wheel motor | wheel [mm] | seed | V | mass [kg] | not recovered | wheel peak [N m] (scenario) | wheel RMS max | wheel sat % max | wheel speed max [rad/s] | v at 1.0 m/s cmd | hip peak (scenario) | hip RMS max | knee peak (scenario) | knee static max | knee RMS max | tire off the floor max [ms] (scenario) | slip max [m/s] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| payload5 | AKE60-8 / AKE60-8 / AKE60-8 | 202 | 0 | 24.0 | 14.931 | none | 12.50 (tilt_25) | 1.99 | 0.5 | 26.9 | 1.04 | 12.50 (push_1.5) | 2.55 | 10.69 (push_1.5) | 7.00 | 7.00 | 36 (slope_15_down_0.5) | 0.25 |
| robot | AKE60-8 / AKE60-8 / AKE60-8 | 202 | 0 | 24.0 | 9.931 | none | 7.06 (tilt_25) | 1.36 | 0.8 | 25.0 | 1.05 | 10.62 (push_1.5) | 1.39 | 7.77 (push_1.5) | 3.87 | 3.88 | 20 (push_1.5) | 0.02 |
