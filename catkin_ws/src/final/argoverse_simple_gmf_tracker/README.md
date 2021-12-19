# Simple Gaussian Mixture Filter (Tracker) argoverse_simple_gmf_tracker

## Summary

This is a modified version of the [Open Argoverse CBGS-KF Tracker](https://github.com/johnwlambert/argoverse_cbgs_kf_tracker) used for the second place submission for the [2020 Argoverse 3D Tracking Competition](https://evalai.cloudcv.org/web/challenges/challenge-page/453/leaderboard/1278). For a more detailed explanation, please refer to the [original repository](https://github.com/johnwlambert/argoverse_cbgs_kf_tracker).

## Changes

Rather than using AB3DMOT's max_age and min_hits parameters, this Gaussian mixture filter uses more conventional parameters for managing Gaussian mixtures ([RFS Example](https://github.com/apak-00/rfs-filters-ssa/blob/master/RFSTest/MixtureModels.cpp)), namely:

 - Each Gaussian component is weighted (w_i, [0,1]), weight is directly proportional to classification score and evolves according to it.
 - Probability of survival (p_s [0,1]) - probability of target survival between a consecutive time steps.
 - Estimate threshold (t_e [0,1]) - threshold above which to report tracked targets.
 - Prune threshold (t_p [0,1]) - threshold below which to remove Gaussian components from the tracker.
 
 - Small part of Mahalanobis distance code is taken from [Probabilistic 3D Multi-Object Tracking for Autonomous Driving](https://github.com/eddyhkchiu/mahalanobis_3d_multi_object_tracking).
 - A model that is closer to the constant velocity model (even not exactly) is used for tracking (delta timestep is taken into account).
 
 Essentially, only probability of survival p_s and estimate threshold needs to be tuned as prune threshold can be set to some low value according to performance considerations.
 
 ## Results on Argoverse Leaderboard


 [Leaderboard](https://evalai.cloudcv.org/web/challenges/challenge-page/453/leaderboard/1278#leaderboardrank-2) classes: (C)ar, (P)edestrian.
 
|  | C:MOTA | P:MOTA | C:MOTPD | P:MOTPD | C:MOTPO | P:MOTPO | C:MOTPI | P:MOTPI | C:IDF1 | P:IDF1 |
|----------------|--------|--------|---------|---------|---------|---------|---------|---------|--------|--------|
| Simple GMF     | 71.54  | 49.62  | 0.33    | 0.36    | 11.60   | 23.20   | 0.18    | 0.18    | 0.81   | 0.60   |
| Baseline       | 65.90  | 48.31  | 0.34    | 0.37    | 15.97   | 25.04   | 0.20    | 0.18    | 0.79   | 0.58   |

## Running the Code

TODO

## Citing this work

Open-source Implementation

@misc{
    author = {Andrey Pak},
    title = {Simple Gaussian Mixture Filter (Tracker)},
    howpublished={\url{https://github.com/apak-00/argoverse_simple_gmf_tracker/}},
    year = {2020},
}
