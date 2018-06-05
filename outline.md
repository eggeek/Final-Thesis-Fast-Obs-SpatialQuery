# Introduction

## Overview

* OkNN in both AI pathfinding and Spatial Query

(Motivation)
With development of robotic, traditional spatial query can not satisfy requirements:

(Application scenario)
* in a warehouse setting, a machine operator...
* in competitive computer games, agent AI ... 

Traditional Euclidean spatial query doesn't consider obstacles;
Road network doesn't exist in this scenario. So we need obstacle spatial query.

## Major Challenges

* Obstacle distance is not easy to compute (fig `pic/obs_dis.png`)

* Drawback of existing works: ...

## Major Objectives
(contributions of our research)

* faster OkNN based on navigation mesh
  * scalability
  * preprocessing
* improve other query processing based on obstacle distance metric
* benchmark for OkNN problem

## Symbols and Definition


## Thesis Organisation


# Literature Review

## Overview

## AI Search

* A*, Dijkstra...



## Spatial indexing

* MBR
* R-tree
* Variants

## OkNN

* History: main memory
* Existing work
  * LVG
  * Fast Filter
* Discussion

## Pathfinding on Navigation Mesh

* History
* Polyanya

## Related Spatial Queries

* Range Query
* ORkNN
* ...

## Summary

# Proposed Algorithms (from SoCS)
## Overview
## Interval Heuristic
## Target Heuristic
## Dam-floodfill preprocessing
## Summary

# Experiments (from SoCS)
## Overview
## Dataset 
## Competitors
## Experiment1: lower bound performance
## Experiment2: performance with increasing k
## Experiment3: performance with increasing t
## Summary

# Conclusion and Future Research
## Research Contribution (take from SoCS)
## Possible ways for improvement (take from SoCS)
## Improve ORkNN
