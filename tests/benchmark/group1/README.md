# Group 1 static benchmark manifest

These tests were converted from legacy map format to the new weighted input format. All terrain costs are `1`; walls are `W`; robot start is moved to the final coordinate line.

| file | energy | free | components | unreachable | maxdist | probe outcome | cov% | steps | returns | recharges | expected |
|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---|
| bench_g1a_density_00.txt | 120 | 400 | 1 | 0 | 38 | SUCCESS | 100.0 | 624 | 6 | 5 | SUCCESS expected under static policy |
| bench_g1a_density_40_irregular.txt | 180 | 274 | 1 | 0 | 39 | SUCCESS | 100.0 | 556 | 4 | 3 | SUCCESS expected under static policy |
| bench_g1b_branch_d50.txt | 220 | 116 | 1 | 0 | 21 | SUCCESS | 100.0 | 166 | 1 | 0 | SUCCESS expected under static policy |
| bench_g1b_branch_d60.txt | 230 | 100 | 1 | 0 | 19 | SUCCESS | 100.0 | 136 | 1 | 0 | SUCCESS expected under static policy |
| bench_g1b_branch_d70.txt | 240 | 92 | 1 | 0 | 19 | SUCCESS | 100.0 | 136 | 1 | 0 | SUCCESS expected under static policy |
| bench_g1c_fragmented_d80.txt | 120 | 150 | 86 | 106 | 38 | FAILED_NO_USABLE_PATH | 29.33 | 82 | 0 | 0 | FAILED_or_PARTIAL due to unreachable fragmented components |
| bench_g1c_fragmented_d90.txt | 120 | 108 | 71 | 85 | 19 | FAILED_NO_USABLE_PATH | 21.3 | 42 | 0 | 0 | FAILED_or_PARTIAL due to unreachable fragmented components |
| bench_g1d_long_snake_infeasible.txt | 120 | 210 | 1 | 0 | 209 | PARTIAL_RETURNED | 24.76 | 102 | 1 | 1 | PARTIAL_RETURNED due to max-energy infeasible long snake |
| bench_g1d_multi_branch_abundant.txt | 230 | 146 | 1 | 0 | 22 | SUCCESS | 100.0 | 170 | 1 | 0 | SUCCESS expected under static policy |
| bench_g1d_multi_branch_power_61.txt | 61 | 146 | 1 | 0 | 22 | SUCCESS | 100.0 | 276 | 6 | 5 | SUCCESS expected under static policy |
| bench_g1d_multi_branch_power_save.txt | 60 | 146 | 1 | 0 | 22 | SUCCESS | 100.0 | 292 | 6 | 5 | SUCCESS expected under static policy |
| bench_g1e_loop_ring_d55.txt | 230 | 150 | 1 | 0 | 24 | SUCCESS | 100.0 | 178 | 1 | 0 | SUCCESS expected under static policy |
| bench_g1f_chokepoint_d60.txt | 230 | 148 | 1 | 0 | 28 | SUCCESS | 100.0 | 196 | 1 | 0 | SUCCESS expected under static policy |
