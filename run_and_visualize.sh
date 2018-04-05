#!/bin/bash
# Input arguments
# 1 - filter distance
# 2 - Ksearch no. neighbors
# 3 - normal dist. wt.
# 4 - dist. threshold planar
# 5 - dist. threshold cylinder
# 6 - maxit planar
# 7 - maxit cylinder
# 9 - input file
cd build/src

./cylinder_segmentation $1 $2 $3 $4 $5 $6 $7 $8 $9

pcl_viewer output_cylinder.pcd
