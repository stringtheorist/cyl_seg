#include <iostream>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include "cylindrical_model.h"

using namespace std;

/*Simple driver program to do cylinder segmentation. 
*/

void usage(char *prog_name) 
{
    std::cerr << "usage:" << std::endl;
    std::cerr << prog_name << " <filter dist> <Ksearch no. neighbors> <normal dist. wt.> <dist. threshold planar> <dist. threshold cylinder> <maxit planar> <maxit cylinder> <radius limit> <input file>" << std::endl;
}

int main(int argc, char *argv[])
{

    //variables to hold input parameters
    float filter_dist;
    int K;
    float normal_dist_wt;
    float planar_thresh;
    float cylinder_thresh;
    int maxit_planar;
    int maxit_cylinder;
    float radius_limit;
    char *input_file;
    char output_plane_file[1000] = "output_plane.pcd";
    char output_cylinder_file[1000] = "output_cylinder.pcd";

    //input argument processing
    if (argc < 10) 
    {
	usage(argv[0]);
	return 1;
    } 
    else 
    {
	filter_dist = atof(argv[1]);
	K = atoi(argv[2]);
	normal_dist_wt = atof(argv[3]);
	planar_thresh = atof(argv[4]);
	cylinder_thresh = atof(argv[5]);
	maxit_planar = atoi(argv[6]);
	maxit_cylinder = atoi(argv[7]);
	radius_limit = atof(argv[8]);
	input_file = argv[9];
    }

    //Do cylinder segmentation:
    //1. Read Input Point Cloud
    //2. build pass through filter to remove nans.
    //3. estimate point normals.
    //4. plane segmentation
    //5. cylinder segmentation
    CylindricalModel model(filter_dist, K, normal_dist_wt, planar_thresh, cylinder_thresh, maxit_planar, maxit_cylinder, radius_limit);
    model.readInCloudData(input_file);
    std::cerr << "Input point cloud has " << model.cloud->points.size() << " points" << std::endl;

    model.buildPassThroughFilter();
    model.estimatePointNormals();
    model.segmentPlanar();

    std::cerr << "Planar component number of points = "<< model.cloud_plane->points.size() << std::endl;
    std::cerr << "Writing planar inliers to " << output_plane_file << std::endl;
    model.writePlanarModel(output_plane_file);

    model.segmentCylinder();
    std::cerr << "Cylinder component points = "<< model.cloud_cylinder->points.size() << std::endl;
    std::cerr << "Writing cylinder inliers to " << output_cylinder_file << std::endl;
    model.writeCylindricalModel(output_cylinder_file);

    return 0;


}
