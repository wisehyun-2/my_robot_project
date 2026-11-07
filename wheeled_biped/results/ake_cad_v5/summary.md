# AKE motor check of the user's robot (`python -m wbr.bench.ake_sizing`)

| motor | peak / rated [N m] | no-load [rpm] (rad/s) | gear | mass [kg] | reflected inertia [kg m^2] | backdrive [N m] |
|---|---|---|---|---|---|---|
| AKE50-8 KV110 | 7.5 / 2.5 | 320 (33.5) | 8:1 | 0.19 | 9.84e-04 | 0.09 |
| AKE60-8 KV80 | 12.5 / 5.0 | 240 (25.1) | 8:1 | 0.26 | 4.14e-03 | 0.12 |

## Per wheel diameter

| case | hip / knee / wheel motor | wheel [mm] | seed | V | mass [kg] | not recovered | wheel peak [N m] (scenario) | wheel RMS max | wheel sat % max | wheel speed max [rad/s] | v at 1.0 m/s cmd | hip peak (scenario) | hip RMS max | knee peak (scenario) | knee static max | knee RMS max | tire off the floor max [ms] (scenario) | slip max [m/s] | longest leg [mm] (scenario) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| payload5 | AKE60-8 / AKE60-8 / AKE60-8 | 202 | 0 | 24.0 | 16.597 | none | 12.50 (tilt_20) | 2.21 | 0.9 | 28.9 | 1.04 | 12.50 (push_1.0) | 3.49 | 12.50 (tilt_15) | 10.03 | 10.08 | 8 (push_1.5) | 0.06 | 487 (height_full_range) |
| robot | AKE60-8 / AKE60-8 / AKE60-8 | 202 | 0 | 24.0 | 11.597 | push_1.5 | 10.47 (tilt_25) | 3.01 | 1.6 | 23.3 | 1.05 | 12.50 (push_1.5) | 9.21 | 11.88 (tilt_25) | 5.85 | 5.89 | 8 (tilt_25) | 0.02 | 488 (height_full_range) |

## Scenarios not recovered

| case | scenario | why | wheel peak / sat % / speed | knee peak / sat % | hip peak / sat % |
|---|---|---|---|---|---|
| robot | push_1.5 | t=1.16 s, pitch -57 deg, height 0.467 m | 5.29 / 1.6 / 23.3 | 6.26 / 0.0 | 12.50 / 52.2 |
