# Segment-resolved held-out report (results/heldout_segments_6laws.csv)

Window = 1.0 turning radius. 12 routes, 18 conditions. RMS cross-track error in metres (exact projection), mean over routes with 95% route-bootstrap interval.

## fixed_wing

Turning share of the path (fraction of samples on arcs): 0.11-0.23

### all conditions

| law | all | straight | transition | arc |
|---|---|---|---|---|
| pure_pursuit | 1.463 [1.190, 1.795] | 0.827 [0.684, 1.004] | 1.942 [1.563, 2.386] | 1.786 [1.507, 2.118] |
| l1 | 1.440 [1.174, 1.765] | 0.804 [0.672, 0.967] | 1.907 [1.534, 2.349] | 1.774 [1.504, 2.098] |
| vector_field | 2.383 [1.897, 2.941] | 0.863 [0.594, 1.184] | 3.044 [2.444, 3.698] | 3.823 [3.241, 4.457] |
| lead_vf | 1.562 [1.271, 1.929] | 0.846 [0.674, 1.070] | 2.064 [1.671, 2.539] | 1.987 [1.727, 2.309] |
| adaptive_vf | 2.386 [1.901, 2.947] | 0.890 [0.623, 1.207] | 3.053 [2.451, 3.714] | 3.821 [3.234, 4.457] |
| carrot_chasing | 1.385 [1.114, 1.724] | 0.389 [0.258, 0.562] | 1.368 [1.014, 1.795] | 2.525 [2.286, 2.814] |

- all: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- straight: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- transition: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- arc: best l1; tie set (paired 95% interval includes 0): l1

### planned at limit (radius x1.0)

| law | all | straight | transition | arc |
|---|---|---|---|---|
| pure_pursuit | 2.112 [1.588, 2.784] | 1.207 [0.874, 1.656] | 3.190 [2.405, 4.155] | 2.633 [2.118, 3.268] |
| l1 | 2.056 [1.549, 2.709] | 1.162 [0.850, 1.581] | 3.104 [2.332, 4.062] | 2.594 [2.099, 3.206] |
| vector_field | 4.550 [3.525, 5.820] | 1.925 [1.229, 2.805] | 6.579 [5.198, 8.162] | 7.202 [6.059, 8.576] |
| lead_vf | 2.238 [1.688, 2.966] | 1.331 [0.943, 1.866] | 3.354 [2.569, 4.366] | 2.848 [2.361, 3.487] |
| adaptive_vf | 4.557 [3.522, 5.846] | 1.967 [1.277, 2.849] | 6.589 [5.200, 8.198] | 7.211 [6.049, 8.615] |
| carrot_chasing | 1.727 [1.210, 2.415] | 0.690 [0.372, 1.143] | 2.346 [1.570, 3.354] | 2.979 [2.523, 3.566] |

- all: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- straight: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- transition: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- arc: best l1; tie set (paired 95% interval includes 0): l1

Whole-path winner is distinguishably worse than the best law in arcs in 4/18 conditions, and on straights in 7/18.

### Effort and saturation (mean over all runs)

| law | sat_fraction | effort (rms cmd / a_max) | not completed |
|---|---|---|---|
| pure_pursuit | 0.024 | 0.259 | 0 |
| l1 | 0.025 | 0.259 | 0 |
| vector_field | 0.105 | 0.381 | 0 |
| lead_vf | 0.031 | 0.274 | 0 |
| adaptive_vf | 0.096 | 0.366 | 0 |
| carrot_chasing | 0.055 | 0.330 | 0 |

## quadcopter

Turning share of the path (fraction of samples on arcs): 0.11-0.23

### all conditions

| law | all | straight | transition | arc |
|---|---|---|---|---|
| pure_pursuit | 0.307 [0.251, 0.377] | 0.166 [0.136, 0.201] | 0.421 [0.338, 0.524] | 0.375 [0.315, 0.443] |
| l1 | 0.302 [0.247, 0.370] | 0.162 [0.134, 0.195] | 0.412 [0.330, 0.514] | 0.371 [0.313, 0.439] |
| vector_field | 0.502 [0.394, 0.635] | 0.194 [0.135, 0.265] | 0.655 [0.517, 0.820] | 0.800 [0.658, 0.962] |
| lead_vf | 0.309 [0.247, 0.389] | 0.182 [0.144, 0.229] | 0.428 [0.339, 0.540] | 0.361 [0.294, 0.442] |
| adaptive_vf | 0.507 [0.400, 0.637] | 0.232 [0.172, 0.301] | 0.655 [0.517, 0.818] | 0.796 [0.654, 0.957] |
| carrot_chasing | 0.278 [0.219, 0.354] | 0.094 [0.063, 0.129] | 0.312 [0.229, 0.419] | 0.454 [0.397, 0.528] |

- all: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- straight: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- transition: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- arc: best lead_vf; tie set (paired 95% interval includes 0): pure_pursuit, l1, lead_vf

### planned at limit (radius x1.0)

| law | all | straight | transition | arc |
|---|---|---|---|---|
| pure_pursuit | 0.486 [0.362, 0.637] | 0.266 [0.192, 0.351] | 0.755 [0.558, 0.986] | 0.589 [0.460, 0.746] |
| l1 | 0.474 [0.354, 0.621] | 0.254 [0.186, 0.333] | 0.735 [0.542, 0.963] | 0.582 [0.456, 0.736] |
| vector_field | 1.018 [0.773, 1.310] | 0.441 [0.285, 0.624] | 1.478 [1.139, 1.865] | 1.610 [1.311, 1.959] |
| lead_vf | 0.522 [0.380, 0.698] | 0.318 [0.224, 0.434] | 0.800 [0.584, 1.053] | 0.604 [0.451, 0.794] |
| adaptive_vf | 1.023 [0.778, 1.310] | 0.501 [0.348, 0.679] | 1.462 [1.125, 1.847] | 1.596 [1.297, 1.945] |
| carrot_chasing | 0.431 [0.298, 0.600] | 0.187 [0.111, 0.275] | 0.635 [0.428, 0.886] | 0.632 [0.497, 0.808] |

- all: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- straight: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- transition: best carrot_chasing; tie set (paired 95% interval includes 0): carrot_chasing
- arc: best l1; tie set (paired 95% interval includes 0): l1, lead_vf

Whole-path winner is distinguishably worse than the best law in arcs in 9/18 conditions, and on straights in 7/18.

### Effort and saturation (mean over all runs)

| law | sat_fraction | effort (rms cmd / a_max) | not completed |
|---|---|---|---|
| pure_pursuit | 0.026 | 0.258 | 0 |
| l1 | 0.026 | 0.260 | 0 |
| vector_field | 0.079 | 0.335 | 0 |
| lead_vf | 0.033 | 0.270 | 0 |
| adaptive_vf | 0.094 | 0.350 | 0 |
| carrot_chasing | 0.053 | 0.314 | 0 |

