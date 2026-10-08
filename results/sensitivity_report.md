# Sensitivity of the headline claims

Each cell: does the claim hold (paired 95% interval over 12 routes) in that world?


## fixed_wing

| condition | perturbation | C1a | C1b | C2a | C2b | C3 | not completed |
|---|---|---|---|---|---|---|---|
| typical (wind .15, r x1.5) | none | yes | yes | yes | NO | NO | 0 |
| typical (wind .15, r x1.5) | lag x0.5 | yes | yes | NO | NO | NO | 0 |
| typical (wind .15, r x1.5) | lag x1.5 | yes | yes | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | lag x2 | yes | NO | yes | yes | yes | 0 |
| typical (wind .15, r x1.5) | true a_max x0.8 | yes | NO | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | true a_max x0.6 | yes | NO | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | position noise 2% r_min | yes | yes | yes | NO | NO | 0 |
| typical (wind .15, r x1.5) | velocity noise 5% | yes | yes | yes | NO | NO | 0 |
| typical (wind .15, r x1.5) | sensor delay 0.2 s | yes | yes | yes | yes | yes | 0 |
| typical (wind .15, r x1.5) | gusts x3 (std 75% of wind) | yes | NO | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | wind 30% | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | none | yes | NO | yes | yes | yes | 0 |
| at limit (wind .15, r x1.0) | lag x0.5 | yes | yes | yes | yes | yes | 0 |
| at limit (wind .15, r x1.0) | lag x1.5 | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | lag x2 | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | true a_max x0.8 | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | true a_max x0.6 | NO | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | position noise 2% r_min | yes | NO | yes | yes | yes | 0 |
| at limit (wind .15, r x1.0) | velocity noise 5% | yes | NO | yes | yes | yes | 0 |
| at limit (wind .15, r x1.0) | sensor delay 0.2 s | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | gusts x3 (std 75% of wind) | yes | NO | yes | yes | yes | 0 |
| at limit (wind .15, r x1.0) | wind 30% | yes | NO | yes | yes | NO | 0 |

Hold rate: C1a 95%, C1b 32%, C2a 95%, C2b 82%, C3 32%

## quadcopter

| condition | perturbation | C1a | C1b | C2a | C2b | C3 | not completed |
|---|---|---|---|---|---|---|---|
| typical (wind .15, r x1.5) | none | yes | yes | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | lag x0.5 | yes | yes | NO | NO | NO | 0 |
| typical (wind .15, r x1.5) | lag x1.5 | yes | yes | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | lag x2 | yes | NO | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | true a_max x0.8 | yes | NO | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | true a_max x0.6 | yes | NO | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | position noise 2% r_min | yes | yes | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | velocity noise 5% | yes | NO | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | sensor delay 0.2 s | yes | yes | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | gusts x3 (std 75% of wind) | yes | NO | yes | yes | NO | 0 |
| typical (wind .15, r x1.5) | wind 30% | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | none | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | lag x0.5 | yes | NO | yes | yes | yes | 0 |
| at limit (wind .15, r x1.0) | lag x1.5 | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | lag x2 | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | true a_max x0.8 | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | true a_max x0.6 | NO | yes | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | position noise 2% r_min | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | velocity noise 5% | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | sensor delay 0.2 s | yes | NO | yes | yes | NO | 0 |
| at limit (wind .15, r x1.0) | gusts x3 (std 75% of wind) | yes | NO | yes | yes | yes | 0 |
| at limit (wind .15, r x1.0) | wind 30% | yes | NO | yes | yes | NO | 0 |

Hold rate: C1a 95%, C1b 27%, C2a 95%, C2b 95%, C3 9%
