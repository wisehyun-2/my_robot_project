# AKE motor check of the user's robot (`python -m wbr.bench.ake_sizing`)

| motor | peak / rated [N m] | no-load [rpm] (rad/s) | gear | mass [kg] | reflected inertia [kg m^2] | backdrive [N m] |
|---|---|---|---|---|---|---|
| AKE50-8 KV110 | 7.5 / 2.5 | 320 (33.5) | 8:1 | 0.19 | 9.84e-04 | 0.09 |
| AKE60-8 KV80 | 12.5 / 5.0 | 240 (25.1) | 8:1 | 0.26 | 4.14e-03 | 0.12 |

## Per wheel diameter

| case | hip / knee / wheel motor | wheel [mm] | seed | V | mass [kg] | not recovered | wheel peak [N m] (scenario) | wheel RMS max | wheel sat % max | wheel speed max [rad/s] | v at 1.0 m/s cmd | hip peak (scenario) | hip RMS max | knee peak (scenario) | knee static max | knee RMS max | tire off the floor max [ms] (scenario) | slip max [m/s] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| payload5 | AKE60-8 / AKE60-8 / AKE60-8 | 150 | 0 | 24.0 | 14.164 | none | 9.10 (tilt_25) | 1.42 | 0.9 | 30.1 | 1.04 | 10.23 (push_1.5) | 3.17 | 10.14 (height_full_range) | 7.30 | 7.29 | 2 (tilt_25) | 0.02 |
| robot | AKE60-8 / AKE60-8 / AKE60-8 | 150 | 0 | 24.0 | 9.164 | none | 5.61 (tilt_25) | 0.95 | 1.1 | 26.9 | 1.05 | 7.79 (push_1.5) | 1.16 | 6.47 (push_1.5) | 4.15 | 4.16 | 6 (tilt_25) | 0.02 |
