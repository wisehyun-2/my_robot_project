# Tire clearance to the leg, upkie layout (`python -m wbr.bench.ake_sizing --clearance`)

Left tire lateral span |y| 132..172 mm: the wheel is inboard of the shank. Smallest gap [mm] between the tire and each part from the MuJoCo collision shapes, Plan 4 leg lengths 0.24..0.335 m / knee angle >= 60 deg; **negative = overlap**. The simulation itself lets the tire pass through the robot (tire-floor contact only).

| part | lateral span |y| [mm] | Ø112 | Ø150 | Ø200 | Ø240 | Ø250 | Ø260 | Ø300 | Ø400 |
|---|---|---|---|---|---|---|---|---|---|
| hip_qdd100_stator | 85..115 | ≥ 100 / 76 | ≥ 100 / 60 | 99 / 42 | 81 / 32 | 76 / 31 | 72 / 29 | 56 / 26 | 31 / 22 |
| hip_qdd100_rotor | 109..114 | ≥ 100 / ≥ 100 | ≥ 100 / 84 | ≥ 100 / 62 | ≥ 100 / 47 | ≥ 100 / 44 | 97 / 41 | 79 / 33 | 41 / 25 |
| hip_horn | 114..164 | ≥ 100 / 77 | ≥ 100 / 59 | 94 / 34 | 74 / 15 | 69 / 10 | 64 / 5 | 45 / **-11** | **-3** / **-24** |
| knee_stator | 121..181 | 69 / 69 | 50 / 50 | 25 / 25 | 5 / 5 | **-0** / **-0** | **-5** / **-5** | **-25** / **-25** | **-44** / **-44** |
| femur | 127..151 | 86 / 85 | 67 / 66 | 42 / 42 | 23 / 22 | 18 / 17 | 13 / 13 | **-3** / **-4** | **-12** / **-13** |
| ankle_mj5208_rotor | 175..191 | 3 / 3 | 3 / 3 | 3 / 3 | 3 / 3 | 3 / 3 | 3 / 3 | 3 / 3 | 3 / 3 |
| ankle_mj5208_stator | 179..197 | 7 / 7 | 7 / 7 | 7 / 7 | 7 / 7 | 7 / 7 | 7 / 7 | 7 / 7 | 7 / 7 |
| knee_qdd100_stator | 181..211 | 74 / 74 | 57 / 57 | 37 / 37 | 25 / 25 | 23 / 23 | 22 / 22 | 17 / 17 | 13 / 13 |
| ankle_plate | 198..202 | 25 / 25 | 26 / 26 | 26 / 26 | 26 / 26 | 26 / 26 | 26 / 26 | 26 / 26 | 25 / 25 |
| ankle_base | 201..257 | 29 / 29 | 29 / 29 | 28 / 28 | 29 / 29 | 29 / 29 | 29 / 29 | 29 / 29 | 29 / 29 |
| knee_qdd100_rotor | 205..210 | ≥ 100 / ≥ 100 | 93 / 93 | 73 / 73 | 60 / 60 | 57 / 57 | 55 / 55 | 47 / 47 | 40 / 40 |
| knee_horn | 210..260 | 92 / 92 | 78 / 78 | 62 / 62 | 54 / 54 | 52 / 52 | 51 / 51 | 47 / 47 | 43 / 43 |
| tibia | 223..247 | 51 / 51 | 51 / 51 | 51 / 51 | 51 / 51 | 51 / 51 | 51 / 51 | 51 / 51 | 51 / 51 |
