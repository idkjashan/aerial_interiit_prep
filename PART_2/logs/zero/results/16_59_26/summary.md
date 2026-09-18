# 16_59_26

| t (s) | event | alt loss (m) | max tilt (°) | max yaw rate (°/s) | motor cmd at end | motor at 100 % (% of time) | outcome |
|---|---|---|---|---|---|---|---|
| 42.7 | motor 1 effectiveness 100% -> 0% | 19.4 | 179 | 190 | 0.00 | 0 | tumbled, ground after 2.2 s |
| 44.9 | motor 1 effectiveness 0% -> 100% | 2.1 | 177 | 170 | 0.76 | 20 | airborne, not settled |

- 42.7 s, motor 1 effectiveness 100% -> 0%: allocator column (roll, pitch, yaw, Tx, Ty, Tz) = [-0.  0.  0.  0.  0. -0.]
- 44.9 s, motor 1 effectiveness 0% -> 100%: allocator column (roll, pitch, yaw, Tx, Ty, Tz) = [-1.43   0.845  0.325  0.     0.    -6.5  ]
