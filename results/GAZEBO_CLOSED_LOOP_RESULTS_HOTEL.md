# Gazebo closed-loop validation (measured)

## Planner: `gazebo_map2_straight_repeated` (n=3 each)

| method | time ms mean ± sample sd | expanded | length m | clearance m* |
|---|---:|---:|---:|---:|
| dijkstra_pp_2602_hotel | 16.881 ± 0.637 | 1025 | 0.950 | 2.500 |
| hybrid_astar_pp_2602_hotel | 0.706 ± 0.108 | 20 | 1.000 | 2.500 |

*8-connected chamfer approximation; not a physical clearance sensor.

## Tracker: `gazebo_map2_straight_tracking` (n=1 each)

| method | success | CTE RMSE m | CTE MAE m | CTE max m | time s | goal err m | mean |Δomega| | omega saturations |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| pure_pursuit_pt_2602_hotel | 1 (goal_tolerance_reached) | 0.012474 | 0.011756 | 0.019694 | 1.974357 | 0.260178 | 0.000326 | 0 |
| lqr_pt_2602_hotel | 1 (goal_tolerance_reached) | 0.015519 | 0.014847 | 0.020000 | 1.754421 | 0.236504 | 0.002078 | 0 |

Tracker results are one reset-controlled trial per controller. They must not be treated as a three-trial mean or as evidence for curves/obstacle scenarios.
