# AKE motor check of the user's robot (`python -m wbr.bench.ake_sizing`)

| motor | peak / rated [N m] | no-load [rpm] (rad/s) | gear | mass [kg] | reflected inertia [kg m^2] | backdrive [N m] |
|---|---|---|---|---|---|---|
| AKE50-8 KV110 | 7.5 / 2.5 | 320 (33.5) | 8:1 | 0.19 | 9.84e-04 | 0.09 |
| AKE60-8 KV80 | 12.5 / 5.0 | 240 (25.1) | 8:1 | 0.26 | 4.14e-03 | 0.12 |

## Per wheel diameter

| case | hip / knee / wheel motor | wheel [mm] | seed | V | mass [kg] | not recovered | wheel peak [N m] (scenario) | wheel RMS max | wheel sat % max | wheel speed max [rad/s] | v at 1.0 m/s cmd | hip peak (scenario) | hip RMS max | knee peak (scenario) | knee static max | knee RMS max | tire off the floor max [ms] (scenario) | slip max [m/s] | longest leg [mm] (scenario) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| payload5 | AKE60-8 / AKE60-8 / AKE60-8 | 202 | 0 | 24.0 | 15.965 | none | 12.50 (tilt_25) | 2.12 | 0.5 | 26.3 | 1.04 | 12.50 (push_1.5) | 2.14 | 11.32 (push_1.5) | 7.41 | 7.44 | 10 (tilt_25) | 0.03 | 338 (height_full_range) |
| robot | AKE60-8 / AKE60-8 / AKE60-8 | 202 | 0 | 24.0 | 10.965 | none | 7.63 (tilt_25) | 1.49 | 0.6 | 24.9 | 1.05 | 11.67 (push_1.5) | 1.31 | 8.54 (push_1.5) | 4.24 | 4.28 | 18 (push_1.5) | 0.04 | 339 (height_full_range) |
