#include "cylindrical_model.h"


CylindricalModel::CylindricalModel(float filter_distance_, int K_, float normal_dist_wt_, float threshold_planar_, 
	float threshold_cylinder_, int maxit_planar_, int maxit_cylinder_, float radius_limit_) : tree (new pcl::search::KdTree<PointT> ()), 
    cloud(new pcl::PointCloud<PointT>),
    cloud_filtered(new pcl::PointCloud<PointT>),
    cloud_normals(new pcl::PointCloud<pcl::Normal>),
    cloud_filtered_nonplane(new pcl::PointCloud<PointT>),
    cloud_normals_nonplane(new pcl::PointCloud<pcl::Normal>),
    coefficients_plane(new pcl::ModelCoefficients),
    coefficients_cylinder(new pcl::ModelCoefficients),
    inliers_plane(new pcl::PointIndices),
    inliers_cylinder(new pcl::PointIndices),
    cloud_cylinder (new pcl::PointCloud<PointT>()),
    cloud_plane (new pcl::PointCloud<PointT>())
{
    filter_distance = filter_distance_;
    K = K_;
    normal_dist_wt = normal_dist_wt_;
    threshold_planar = threshold_planar_;
    threshold_cylinder = threshold_cylinder_;
    maxit_planar = maxit_planar_;
    maxit_cylinder = maxit_cylinder_;
    radius_limit = radius_limit_;
}

void CylindricalModel::readInCloudData(char *filename)
{
    // Read in the cloud data
    reader.read (filename, *cloud);
}

void CylindricalModel::buildPassThroughFilter(void)
{
    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, filter_distance);
    pass.filter (*cloud_filtered);
}

void CylindricalModel::estimatePointNormals(void)
{
    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (K);
    ne.compute (*cloud_normals);
}

void CylindricalModel::segmentPlanar(void)
{
    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (normal_dist_wt);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxit_planar);
    seg.setDistanceThreshold (threshold_planar);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    //Populate data structure with plane inliers
    extract.filter (*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered_nonplane);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals_nonplane);
}

void CylindricalModel::writePlanarModel(char *filename)
{
    // Write the planar inliers to disk
    if (cloud_plane->points.empty ()) 
	std::cerr << "Planar component empty!" << std::endl;
    else
    {
	std::cerr << "Writing planar component point cloud with size " << cloud_plane->points.size () << " data points." << std::endl;
	writer.write (filename, *cloud_plane, false);
    }
    writer.write (filename, *cloud_plane, false);
}

void CylindricalModel::segmentCylinder(void)
{
    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (normal_dist_wt);
    seg.setMaxIterations (maxit_cylinder);
    seg.setDistanceThreshold (threshold_cylinder);
    seg.setRadiusLimits (0, radius_limit);
    seg.setInputCloud (cloud_filtered_nonplane);
    seg.setInputNormals (cloud_normals_nonplane);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    // Populate final point cloud representing cylinder inliers
    extract.setInputCloud (cloud_filtered_nonplane);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud_cylinder);
}

void CylindricalModel::writeCylindricalModel(char *filename)
{
    //write cylinder inliers to disk
    if (cloud_cylinder->points.empty ()) 
	std::cerr << "Cylindrical component empty!" << std::endl;
    else
    {
	std::cerr << "Writing cylindrical component point cloud with size " << cloud_cylinder->points.size () << " data points." << std::endl;
	writer.write (filename, *cloud_cylinder, false);
    }
}

//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
//  viewer->addPointCloud<pcl::PointXYZ> (cloud_cylinder, "sample cloud");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//  viewer->addCoordinateSystem (1.0);
//  viewer->initCameraParameters ();
//
//  while(!viewer->wasStopped()) 
//  {
//    viewer->spinOnce(100);
//    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//  }
