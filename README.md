
README for CYL_SEG
------------------

This is a simple driver program for running a Sample Consensus segmentation for cylindrical models.

I. COMPILATION AND INSTALLATION
-------------------------------

I.i Prerequisites: pcl (including command line tools, and all its dependencies), cmake (version 3.11), 
                   internet connection (for downloading Gtest). 

I.ii Procedure:
1. Change to the top level directory of the project. 
2. execute: mkdir build && cd build
3. execute: cmake ..
4. execute: make

I.ii Notes: 
1. The unit testing framework GTest will be downloaded during the build process, this requires an internet connection.
2. A reference implementation will be built and run during the build process to generate data for the test cases to use 
   in comparisons. 
3. The main executable "cylinder_segmentation" will be found in the "src" subdirectory of the "build" directory.

II. RUNNING THE TESTS
---------------------

Execute the following command from the build directory after building the project to run the tests:

$make test

If you desire verbose output from the tests then you can alternatively run

$make test ARGS="-V"

III. USAGE
----------

./cylinder segmentation <filter dist> <Ksearch no. neighbors> <normal dist. wt.> <dist. threshold planar> <dist. threshold cylinder> <maxit planar> <maxit cylinder> <radius limit> <input file>

<filter dist> : points beyond this distance in m are filtered.
<Ksearch no. neighbors> : Number of neighbors to use for surface normals estimation of the input point cloud.
<normal dist. wt.> : The surface normals' influence parameter.
<dist. threshold planar> : distance threshold from plane inlier points to planar model. (m)
<dist. threshold cylinder> : distance threshold from cylinder inlier points to the cylindrical model. (m)
<maxit planar> : Maximum iterations for segmentation for generation of planar model.
<maxit cylinder> : Maximum iterations for segmentation for generation of cylindrical model.  
<input file> : input .pcd file for cylinder segmentation.

III.i Notes: 
1. The program writes two output files output_plane.pcd, and output_cylinder.pcd for the output of plane segmentation (planar inliers)  and cylinder segmentation (cylinder inliers).
2. The output files are in the .pcd format. 


IV. VISUALIZATION
-----------------

There are two options. One can change directories to the top level directory of the project and 
run the script run_and_visualize.sh with the same arguments as the cylinder_segmentation executable.
Otherwise one can use pcl_viewer directly, or the visualize.sh script with the output pcd file 
as argument to visualize the result. Note that this requires pcl tools to be installed and pcl_viewer
to be in the PATH.  
