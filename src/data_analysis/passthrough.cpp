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
    //pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    //pcl::NormalEstimation<PointT, pcl::Normal> ne;
    //pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    //pcl::PCDWriter writer;
    //pcl::ExtractIndices<PointT> extract;
    //pcl::ExtractIndices<pcl::Normal> extract_normals;
    //pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    
    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::IndicesPtr indices (new std::vector<int>);
    //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    //pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    //pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    
    // Read in the cloud data
    reader.read ("test_pcd.pcd", *cloud);
    std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
    
    /*
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("test_pcd.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    */

    // ### FILTERING ###
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-7.0, 2.0);
    pass.filter (*indices);
    pass.setIndices (indices);
    
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.8, 2.0);
    pass.filter (*indices);
    pass.setIndices (indices);
    
    pass.setFilterFieldName ("intensity");
    pass.setFilterLimits (FLT_MIN, FLT_MAX);
    pass.setNegative (false);
    pass.filter (*cloud_filtered);
    
    pcl::io::savePCDFileBinaryCompressed ("test_filtered_pcd.pcd", *cloud_filtered);

    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.2, 0.2);
   
    pass.filter (*cloud_filtered);
    
    pcl::io::savePCDFileBinaryCompressed ("test_fixedZ_pcd.pcd", *cloud_filtered);

    return (0);
}
