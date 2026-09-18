# 16_56_51

| t (s) | event | alt loss (m) | max tilt (°) | max yaw rate (°/s) | motor cmd at end | motor at 100 % (% of time) | outcome |
|---|---|---|---|---|---|---|---|
| 62.7 | motor 1 effectiveness 100% -> 75% | 0.0 | 7 | 10 | 0.73 | 0 | holds |
| 82.7 | motor 1 effectiveness 75% -> 100% | 0.1 | 7 | 9 | 0.73 | 0 | holds |
| 88.3 | motor 1 effectiveness 100% -> 50% | 0.0 | 17 | 48 | 0.75 | 0 | airborne, not settled |
| 108.4 | motor 1 effectiveness 50% -> 100% | 0.4 | 24 | 37 | 0.73 | 0 | holds |
| 116.1 | motor 1 effectiveness 100% -> 25% | 19.7 | 25 | 35 | 0.61 | 1 | came down upright, ground after 3.6 s |
| 119.8 | motor 1 effectiveness 25% -> 100% | 1.2 | 58 | 319 | 0.75 | 12 | airborne, not settled |

- 62.7 s, motor 1 effectiveness 100% -> 75%: allocator column (roll, pitch, yaw, Tx, Ty, Tz) = [-1.072  0.634  0.244  0.     0.    -4.875]
- 82.7 s, motor 1 effectiveness 75% -> 100%: allocator column (roll, pitch, yaw, Tx, Ty, Tz) = [-1.43   0.845  0.325  0.     0.    -6.5  ]
- 88.3 s, motor 1 effectiveness 100% -> 50%: allocator column (roll, pitch, yaw, Tx, Ty, Tz) = [-0.715  0.422  0.163  0.     0.    -3.25 ]
- 108.4 s, motor 1 effectiveness 50% -> 100%: allocator column (roll, pitch, yaw, Tx, Ty, Tz) = [-1.43   0.845  0.325  0.     0.    -6.5  ]
- 116.1 s, motor 1 effectiveness 100% -> 25%: allocator column (roll, pitch, yaw, Tx, Ty, Tz) = [-0.357  0.211  0.081  0.     0.    -1.625]
- 119.8 s, motor 1 effectiveness 25% -> 100%: allocator column (roll, pitch, yaw, Tx, Ty, Tz) = [-1.43   0.845  0.325  0.     0.    -6.5  ]
