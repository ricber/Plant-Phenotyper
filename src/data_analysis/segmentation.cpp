#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZI PointT;

int main (int argc, char** argv)
{
    // All the objects needed
    pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    
    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::IndicesPtr indices (new std::vector<int>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients), coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices), inliers_plane (new pcl::PointIndices);

    
    // Read in the cloud data
    reader.read ("test_pcd.pcd", *cloud);
    std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
    

    // ### FILTERING ###
    // remove lateral points outside the vineyard row
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-7.0, 2.0);
    pass.filter (*indices);
    pass.setIndices (indices);
    
    // remove ground points and points above the plants
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.7, -0.17);
    pass.filter (*indices);
    pass.setIndices (indices);

    // remove low intensity points
    pass.setFilterFieldName ("intensity");
    pass.setFilterLimits (FLT_MIN, FLT_MAX); // now is deactivated
    pass.setNegative (false);
    pass.filter (*cloud_filtered);
    
    writer.write ("test_filtered_pcd.pcd", *cloud_filtered, true); // the boolean flag is for binary (true) or ASCII (false) file format
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
    
    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
    
    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.17);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    if (inliers_plane->indices.size () == 0)
    { 
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

     // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);
    
    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    writer.write ("test_plane_pcd.pcd", *cloud_plane, false);
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);
    
    std::cerr << "PointCloud representing the cylindrical components: " << cloud_filtered2->points.size () << " data points." << std::endl;
    writer.write ("test_filtered2_pcd.pcd", *cloud_filtered2, false);
   
    /*
    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
    
    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true); // Set to true if a coefficient refinement by least square method is required
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.03); // radius of the cylinder
    seg.setInputCloud (cloud_filtered); 
    seg.setInputNormals (cloud_normals);
    
    extract.setInputCloud (cloud_filtered);

    // segment until there are no remaining cylinders
    pcl::PointCloud<PointT>::Ptr temp_cylinder (new pcl::PointCloud<PointT>()), cloud_cylinder (new pcl::PointCloud<PointT>());
    do
    {
        // Obtain the cylinder inliers and coefficients
        seg.segment (*inliers_cylinder, *coefficients_cylinder);
        std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

        // Write the cylinder inliers to disk
        extract.setNegative (false);
        extract.setIndices (inliers_cylinder);
        
        // add the found cylinder to a point cloud
        extract.filter (*temp_cylinder);
        // const pcl::PointCloud<PointT> *temp_cyl_ptr = temp_cylinder;
        *cloud_cylinder = cloud_cylinder->operator+=(*temp_cylinder);
        
        // set the extract object to remove the last found cylinder
        extract.setNegative (true);
        extract.filter (*cloud_filtered);
        extract_normals.setNegative (true);
        extract_normals.setInputCloud (cloud_normals);
        extract_normals.setIndices (inliers_cylinder);
        extract_normals.filter (*cloud_normals);
        
        // set the new clouds without the found cylinder
        seg.setInputCloud (cloud_filtered); 
        seg.setInputNormals (cloud_normals);
        extract.setInputCloud (cloud_filtered);
    }
    while (!temp_cylinder->points.empty ());
   
    if (cloud_cylinder->points.empty ())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
        writer.write ("test_cylinders.pcd", *cloud_cylinder, false);
    }
    */
    return (0);
}
