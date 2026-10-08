# Planner vs controller (whole-path RMS, metres)

## quadcopter

| law | Dubins | smooth (Dubins gains) | smooth (own gains) |
|---|---|---|---|
| pure_pursuit | 0.307 | 0.207 | 0.185 |
| l1 | 0.302 | 0.205 | 0.184 |
| vector_field | 0.502 | 0.181 | 0.175 |
| lead_vf | 0.309 | 0.226 | 0.178 |
| adaptive_vf | 0.507 | 0.183 | 0.189 |
| carrot_chasing | 0.278 | 0.190 | 0.171 |

- controller effect on dubins routes, dubins-tuned: spread 0.229 m [0.175, 0.288] (best carrot_chasing, worst adaptive_vf); ranking: carrot_chasing < l1 < pure_pursuit < lead_vf < vector_field < adaptive_vf
- controller effect on smooth routes, dubins-tuned: spread 0.045 m [0.033, 0.056] (best vector_field, worst lead_vf); ranking: vector_field < adaptive_vf < carrot_chasing < l1 < pure_pursuit < lead_vf
- controller effect on smooth routes, own-tuned: spread 0.018 m [0.002, 0.035] (best carrot_chasing, worst adaptive_vf); ranking: carrot_chasing < vector_field < lead_vf < l1 < pure_pursuit < adaptive_vf
- planner effect (Dubins -> smooth, dubins-tuned): mean |change| 0.169 m [0.133, 0.209]; mean signed change -0.169 m
- planner effect (Dubins -> smooth, own-tuned): mean |change| 0.187 m [0.154, 0.226]; mean signed change -0.187 m

## fixed_wing

| law | Dubins | smooth (Dubins gains) | smooth (own gains) |
|---|---|---|---|
| pure_pursuit | 1.463 | 1.060 | 0.853 |
| l1 | 1.440 | 1.049 | 0.861 |
| vector_field | 2.383 | 0.844 | 0.911 |
| lead_vf | 1.562 | 1.143 | 0.802 |
| adaptive_vf | 2.386 | 0.869 | 0.844 |
| carrot_chasing | 1.385 | 1.099 | 0.782 |

- controller effect on dubins routes, dubins-tuned: spread 1.001 m [0.757, 1.268] (best carrot_chasing, worst adaptive_vf); ranking: carrot_chasing < l1 < pure_pursuit < lead_vf < vector_field < adaptive_vf
- controller effect on smooth routes, dubins-tuned: spread 0.299 m [0.240, 0.355] (best vector_field, worst lead_vf); ranking: vector_field < adaptive_vf < l1 < pure_pursuit < carrot_chasing < lead_vf
- controller effect on smooth routes, own-tuned: spread 0.129 m [0.076, 0.186] (best carrot_chasing, worst vector_field); ranking: carrot_chasing < lead_vf < adaptive_vf < pure_pursuit < l1 < vector_field
- planner effect (Dubins -> smooth, dubins-tuned): mean |change| 0.759 m [0.627, 0.899]; mean signed change -0.759 m
- planner effect (Dubins -> smooth, own-tuned): mean |change| 0.927 m [0.813, 1.046]; mean signed change -0.927 m

