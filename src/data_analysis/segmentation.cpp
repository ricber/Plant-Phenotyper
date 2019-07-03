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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/ml/kmeans.h>

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

    
    // Read in the cloud data
    reader.read ("test_pcd.pcd", *cloud);
    std::cout << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

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
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
    
    // Create the outlier filtering object
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
    
    writer.write ("test_outlier_removed_pcd.pcd", *cloud_filtered, true); // the boolean flag is for binary (true) or ASCII (false) file format
    std::cout << "PointCloud after ourlier removal has: " << cloud_filtered->points.size () << " data points." << std::endl;
    
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
    std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;

     // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);
    
    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    writer.write ("test_plane_pcd.pcd", *cloud_plane, false);
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);
    
    std::cout << "PointCloud representing the cylindrical components: " << cloud_filtered2->points.size () << " data points." << std::endl;
    writer.write ("test_filtered2_pcd.pcd", *cloud_filtered2, false);
    
     // Creating the KdTree object for the search method of the extraction
    tree->setInputCloud (cloud_filtered2);

    // Euclidean Cluster Extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.05); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered2);
    ec.extract (cluster_indices);

    std::vector <pcl::PointIndices> clusters_ = cluster_indices;
    pcl::PointCloud<PointT>::Ptr input_ = cloud_filtered2;
    std::vector< pcl::Kmeans::Point > centroids(cluster_indices.size());
    if (!clusters_.empty ())
    {
        colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

        srand (static_cast<unsigned int> (time (0)));
        std::vector<unsigned char> colors;
        for (size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
        {
            colors.push_back (static_cast<unsigned char> (rand () % 256));
            colors.push_back (static_cast<unsigned char> (rand () % 256));
            colors.push_back (static_cast<unsigned char> (rand () % 256));
        }
        
        colored_cloud->width = input_->width;
        colored_cloud->height = input_->height;
        colored_cloud->is_dense = input_->is_dense;
        for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
        {
            pcl::PointXYZRGB point;
            point.x = *(input_->points[i_point].data);
            point.y = *(input_->points[i_point].data + 1);
            point.z = *(input_->points[i_point].data + 2);
            point.r = 255;
            point.g = 0;
            point.b = 0;
            colored_cloud->points.push_back (point);
        }
        
        std::vector< pcl::PointIndices >::iterator i_segment;
        int next_color = 0;
        for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
        {
            std::vector<int>::iterator i_point;
            for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
            {
                int index;
                index = *i_point;
                colored_cloud->points[index].r = colors[3 * next_color];
                colored_cloud->points[index].g = colors[3 * next_color + 1];
                colored_cloud->points[index].b = colors[3 * next_color + 2];
            }
            
            
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid (*cloud_filtered2, i_segment->indices, centroid);
            std::vector<float> & centroid_vector = centroids[next_color];
            for (int i = 0; i < 3; i++) {
                centroid_vector.push_back (centroid[i]);
            }  
            
            /* // DEBUG
            std::cout << next_color << "_cent output: x: " << centroids[next_color][0] << " ,";
            std::cout << "y: " << centroids[next_color][1] << " ,";
            std::cout << "z: " << centroids[next_color][2] << std::endl;
            }  
            */       
            
            next_color++;       
        }
    }

    std::cout << "PointCloud representing the clusters: " << colored_cloud->points.size () << " data points." << std::endl;
    writer.write ("test_colored_cloud_pcd.pcd", *colored_cloud, false); 
    
    // Kmeans computation
    pcl::Kmeans kmeans(static_cast<int> (cloud_filtered2->points.size()), 3);
    kmeans.setClusterSize(cluster_indices.size());
    kmeans.setClusterCentroids(centroids);
    // ADD DATA POINTS TO KMEANS!
      
    kmeans.kMeans();
    
    pcl::Kmeans::Centroids ret_centroids = kmeans.get_centroids();    
    /* // DEBUG
     std::cout << "centroid count: " << ret_centroids.size() << std::endl;
        for (int i = 0; i<ret_centroids.size(); i++)
        {
            std::cout << i << "_cent output: x: " << ret_centroids[i][0] << " ,";
            std::cout << "y: " << ret_centroids[i][1] << " ,";
            std::cout << "z: " << ret_centroids[i][2] << std::endl;
        }
    */
    
   
    return (0);
}
