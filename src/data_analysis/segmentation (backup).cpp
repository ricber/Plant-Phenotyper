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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/io.h>

typedef pcl::PointXYZI PointT;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud (pcl::PointCloud<PointT>& input, std::vector <pcl::PointIndices>& clusters);
pcl::PointCloud<PointT>::Ptr filter (pcl::PointCloud<PointT>::Ptr input);
void remove_ground (pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr output, pcl::ModelCoefficients::Ptr coefficients_plane);
pcl::PointCloud<PointT>::Ptr filter_z (pcl::PointCloud<PointT>::Ptr input);
std::vector<pcl::PointIndices> cluster_trunks (pcl::PointCloud<PointT>::Ptr input);
std::vector<pcl::Kmeans::Point> kmeans_centroids_computation(pcl::PointCloud<PointT>::Ptr input, std::vector<pcl::PointIndices> &input_clusters_indices);
std::vector<pcl::PointIndices> cluster_kmeans (pcl::PointCloud<PointT>::Ptr input, std::vector<pcl::Kmeans::Point> &centroids);
pcl::PointCloud<PointT>::Ptr transform (pcl::PointCloud<PointT>::Ptr input);
void visualize (pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<PointT>::Ptr cloud_transformed);
void boundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented, pcl::visualization::PCLVisualizer *visu, int counter);
pcl::PointCloud<PointT>::Ptr cloud_projection (pcl::PointCloud<PointT>::Ptr input, pcl::ModelCoefficients::Ptr coefficients);
std::vector<pcl::PointIndices> cluster_radius_search (pcl::PointCloud<PointT>::Ptr cloud_projected, pcl::PointCloud<PointT>::Ptr centroids_projected, std::vector<std::vector<int>> &pointIdxRadiusSearchClusters, std::vector<std::vector<float>> &pointRadiusSquaredDistanceClusters);
pcl::PointCloud<PointT>::Ptr variance_filtering(std::vector<std::vector<float>> &pointRadiusSquaredDistanceClusters, std::vector<pcl::PointIndices> &circle_clusters_indices, pcl::PointCloud<PointT>::Ptr cloud_filtered2, std::vector< pcl::Kmeans::Point > &centroids, std::vector<pcl::Kmeans::Point> &centroids_filtered);



int main (int argc, char** argv)
{
    // All the objects needed
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    std::vector<pcl::PointIndices> trunks_clusters_indices;
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

    
    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered, trunks_filtered, cloud_transformed, final_cloud, cloud_projected, centroids_projected; // (new pcl::PointCloud<PointT>)
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>), cloud_filtered3 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trunks_colored, final_colored_cloud, circles_colored;

    
    
    // Read in the cloud data
    reader.read ("input.pcd", *cloud);
    std::cout << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;



    // ########## AFFINE TRANSFORMATION ########## 
    cloud_transformed = transform (cloud);

    std::cout << "PointCloud after transformation has: " << cloud_transformed->points.size () << " data points." << std::endl;
    writer.write ("cloud_transformed.pcd", *cloud_transformed, true); // the boolean flag is for binary (true) or ASCII (false) file format
        
        
        
    // ########## VISUALIZATION ########## 
    //visualize (cloud, cloud_transformed); // Display the visualiser until 'q' key is pressed
        
        

    // ########## FILTERING ##########
    cloud_filtered = filter(cloud_transformed);
    
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
    writer.write ("cloud_filtered.pcd", *cloud_filtered, true); // the boolean flag is for binary (true) or ASCII (false) file format
    
    
    
    // ########## GROUND REMOVAL ##########
    remove_ground(cloud_filtered, cloud_filtered2, coefficients_plane);
    if(cloud_filtered2->empty())
        return(-1);
    
    std::cout << "PointCloud after ground removal has: " << cloud_filtered2->points.size () << " data points." << std::endl;
    writer.write ("cloud_filtered2.pcd", *cloud_filtered2, true);
    
    
    
    // ########## Z FILTERING: TRUNKS ##########
    trunks_filtered = filter_z (cloud_filtered2);
    
    std::cout << "PointCloud after Z filtering has: " << trunks_filtered->points.size () << " data points." << std::endl;
    writer.write ("trunks_filtered.pcd", *trunks_filtered, true); // the boolean flag is for binary (true) or ASCII (false) file format
    
    
    
    // ########## TRUNKS CLUSTERING ##########
    trunks_clusters_indices = cluster_trunks (trunks_filtered);
    
    // color cloud with different colors for each cluster
    std::size_t num_clusters = trunks_clusters_indices.size();
    trunks_colored = getColoredCloud (*trunks_filtered, trunks_clusters_indices);
    
    std::cout << "PointCloud after Euclidean Cluster Extraction has: " << trunks_colored->points.size () << " data points." << std::endl;
    writer.write ("trunks_colored.pcd", *trunks_colored, true); // the boolean flag is for binary (true) or ASCII (false) file format
    
    
    
    // ########## (KMEANS) CENTROIDS COMPUTATION ##########   
    std::vector< pcl::Kmeans::Point > centroids(num_clusters); 
    centroids = kmeans_centroids_computation(trunks_filtered, trunks_clusters_indices);
    
    
    
    // ########## CLOUD PROJECTION ##########
    cloud_projected = cloud_projection (cloud_filtered2, coefficients_plane);
     
    std::cout << "PointCloud after Cloud Projection has: " << cloud_projected->points.size () << " data points." << std::endl;
    writer.write ("cloud_projected.pcd", *cloud_projected, true); // the boolean flag is for binary (true) or ASCII (false) file format 
    
    
    
    // ########## CENTROIDS PROJECTION ##########
    pcl::PointCloud<PointT>::Ptr centroids_cloud (new pcl::PointCloud<PointT>);

    // Fill in the cloud data
    centroids_cloud->width    = num_clusters;
    centroids_cloud->height   = 1;
    centroids_cloud->is_dense = true;
    centroids_cloud->points.resize (centroids_cloud->width * centroids_cloud->height);

    for (size_t i = 0; i < centroids_cloud->points.size (); ++i)
    {
        centroids_cloud->points[i].x = centroids[i].at(0);
        centroids_cloud->points[i].y = centroids[i].at(1);
        centroids_cloud->points[i].z = centroids[i].at(2);
        centroids_cloud->points[i].intensity = 0.0f;
    } 
     
    centroids_projected = cloud_projection (centroids_cloud, coefficients_plane);        
    
    std::cout << "PointCloud after Centroids Cloud Projection has: " <<  centroids_projected->points.size () << " data points." << std::endl;
    writer.write ("centroids_projected.pcd", *centroids_projected, true); // the boolean flag is for binary (true) or ASCII (false) file format 
    
    
    
    // ########## RADIUS SEARCH ##########    
    std::vector<std::vector<int>> pointIdxRadiusSearchClusters (num_clusters);
    std::vector<std::vector<float>> pointRadiusSquaredDistanceClusters (num_clusters);
    std::vector<pcl::PointIndices> circle_clusters_indices (num_clusters);
    circle_clusters_indices = cluster_radius_search (cloud_projected, centroids_projected, pointIdxRadiusSearchClusters, pointRadiusSquaredDistanceClusters);    
    
    circles_colored = getColoredCloud(*cloud_projected, circle_clusters_indices);
    
    std::cout << "PointCloud representing the circles clusters: " <<  circles_colored->points.size () << " data points." << std::endl;
    writer.write ("circles_colored.pcd", *circles_colored, true); // the boolean flag is for binary (true) or ASCII (false) file format
    
    
    
    // ########## VARIANCE FILTERING ########## 
    std::vector<pcl::Kmeans::Point> centroids_filtered;
    cloud_filtered3 = variance_filtering(pointRadiusSquaredDistanceClusters, circle_clusters_indices, cloud_filtered2, centroids, centroids_filtered);

    
    std::cout << "PointCloud after Variance Filtering has: " <<  cloud_filtered3->points.size () << " data points." << std::endl;
    writer.write ("cloud_filtered3.pcd", *cloud_filtered3, true); // the boolean flag is for binary (true) or ASCII (false) file format
    
    
    
    final_cloud = cloud_filtered3;
    
    
   
    // ########## KMEANS CLUSTERING ##########    
    std::vector<pcl::PointIndices> plants_clusters_indices (centroids_filtered.size());
    plants_clusters_indices = cluster_kmeans(final_cloud, centroids_filtered);
    final_colored_cloud = getColoredCloud(*final_cloud, plants_clusters_indices);
    
    std::cout << "PointCloud representing the kmeans plants clusters: " << final_colored_cloud->points.size () << " data points." << std::endl;
    writer.write ("final_colored_cloud.pcd", *final_colored_cloud, true); // the boolean flag is for binary (true) or ASCII (false) file format
    
    
    
    // ########## BOUNDING BOXES ##########
    std::vector<pcl::PointIndices>::iterator it_1 = plants_clusters_indices.begin(); // iterate over clusters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>); // cloud of a single cluster
    pcl::ExtractIndices<pcl::PointXYZRGB> extract; // object to extract cluster points
    pcl::visualization::PCLVisualizer *visu;  // visualization object
    
    extract.setInputCloud (final_colored_cloud);
    visu = new pcl::visualization::PCLVisualizer (argc, argv, "Bound boxes Viewer");  
    
    int counter=0;
    // Iterate till the end of the vector of clusters
    while (it_1 != plants_clusters_indices.end())
    {
        pcl::PointIndices::Ptr cluster_indices (new pcl::PointIndices ()); // point indices of a single cluster
        std::vector< int >::iterator it_2 = it_1->indices.begin();
        while (it_2 != it_1->indices.end())
        {
            cluster_indices->indices.push_back(*it_2);
            it_2++;        
        }
        
        extract.setIndices (cluster_indices);
        extract.setNegative (false);
        extract.filter (*cloud_cluster);
        boundingBox(cloud_cluster, visu, counter);       
        it_1++;
        counter++;
    }
    
    visu->setRepresentationToWireframeForAllActors(); 
    
    while (!visu->wasStopped ()) { // Display the visualiser until 'q' key is pressed
        visu->spinOnce ();
    }

    
   
    return (0);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// ########## VARIANCE FILTERING ########## 
pcl::PointCloud<PointT>::Ptr variance_filtering(std::vector<std::vector<float>> &pointRadiusSquaredDistanceClusters, std::vector<pcl::PointIndices> &circle_clusters_indices, pcl::PointCloud<PointT>::Ptr cloud_filtered2, std::vector< pcl::Kmeans::Point > &centroids, std::vector<pcl::Kmeans::Point> &centroids_filtered)
{
    pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);
    float circle_variance;
    const float circvar_thr = 0.0115f;
    pcl::ExtractIndices<PointT> extract;
    std::set<int> indices_set;
    size_t num_clusters = circle_clusters_indices.size();
    
    
    for (int i=0; i < num_clusters; i++)
    {
        for (std::vector<float>::iterator iterator = pointRadiusSquaredDistanceClusters[i].begin(); iterator != pointRadiusSquaredDistanceClusters[i].end(); iterator++)
        {
            circle_variance += *iterator;
        }
        
        circle_variance /= pointRadiusSquaredDistanceClusters[i].size();
        
        std::cout << "Variance of circle cluster #" <<  std::to_string(i+1) << " is: " << std::to_string(circle_variance) << std::endl;
        
        if (circle_variance < circvar_thr)
        {
            for (std::vector<int>::iterator index = circle_clusters_indices[i].indices.begin(); index != circle_clusters_indices[i].indices.end(); index++)
            {
                indices_set.insert(*index);
            }
        }
        else
        {
            centroids_filtered.push_back(centroids[i]);
        }        
    }
    
    pcl::PointIndices::Ptr pi_ptr (new pcl::PointIndices);
    std::vector<int> indices_vector(indices_set.begin(), indices_set.end());
    pi_ptr->indices = indices_vector;
    extract.setInputCloud(cloud_filtered2);
    extract.setNegative(true);
    extract.setIndices(pi_ptr);
    extract.filter(*output);
    
    return(output);
}



// ########## RADIUS SEARCH ########## 
std::vector<pcl::PointIndices> cluster_radius_search (pcl::PointCloud<PointT>::Ptr cloud_projected, pcl::PointCloud<PointT>::Ptr centroids_projected, std::vector<std::vector<int>> &pointIdxRadiusSearchClusters, std::vector<std::vector<float>> &pointRadiusSquaredDistanceClusters)
{   
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (cloud_projected);
    size_t num_clusters = centroids_projected->points.size();
    
    
    
    // Neighbors within radius search
    int i = 0;
    for (pcl::PointCloud<PointT>::iterator cloud_iterator = centroids_projected->begin (); cloud_iterator != centroids_projected->end (); cloud_iterator++)
    {
        
        PointT searchPoint;

        searchPoint.x = cloud_iterator->x;
        searchPoint.y = cloud_iterator->y;
        searchPoint.z = cloud_iterator->z;
        searchPoint.intensity = cloud_iterator->intensity;
    
        float radius = 0.22f;

        kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearchClusters[i], pointRadiusSquaredDistanceClusters[i]); // some clusters can overlap
        
        i++;
    }
    
    std::vector<pcl::PointIndices> circle_clusters_indices (num_clusters);
    for (size_t i = 0; i < num_clusters; i++)
    {
            std::vector<int>::iterator it = pointIdxRadiusSearchClusters[i].begin();
 
            // Iterate till the end of vector
            while (it != pointIdxRadiusSearchClusters[i].end())
            {
                circle_clusters_indices[i].indices.push_back(static_cast<int>(*it));
                it++;
            }
    }  
    
    return(circle_clusters_indices);
}



// ########## CLOUD PROJECTION ##########
pcl::PointCloud<PointT>::Ptr cloud_projection (pcl::PointCloud<PointT>::Ptr input, pcl::ModelCoefficients::Ptr coefficients)
{
    pcl::PointCloud<PointT>::Ptr cloud_projected (new pcl::PointCloud<PointT>);
    pcl::ProjectInliers<PointT> proj;
    
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (input);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    
    return(cloud_projected);
}



// ########## BOUNDING BOXES ##########
void boundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented, pcl::visualization::PCLVisualizer *visu, int counter)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    // This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    // the signs are different and the box doesn't get correctly oriented in some cases.
    //eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
    //eigenVectorsPCA.col(2) =  Eigen::Vector3f::UnitZ(); // actually I am not sure the Z component is always in the third column
    eigenVectorsPCA =  Eigen::Matrix3f::Identity();                                                                            
                                                                                   
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    
    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    
    visu->addPointCloud(cloudSegmented, "cloud" + std::to_string(counter));
    visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "cube" + std::to_string(counter));
    visu->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud" + std::to_string(counter));
    //visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube_name); 
}



// ########## VISUALIZATION ########## 
void visualize (pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<PointT>::Ptr cloud_transformed)
{  
    // Visualization
    printf( "\nPoint cloud colors :  white  = original point cloud\n"
        "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer ("Point cloud rotation");
    
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler (source_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed_cloud_color_handler (cloud_transformed, 230, 20, 20); // Red
    viewer.addPointCloud (cloud_transformed, transformed_cloud_color_handler, "cloud_transformed");
    
    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_transformed");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }
}



// ########## AFFINE TRANSFORMATION ########## 
pcl::PointCloud<PointT>::Ptr transform (pcl::PointCloud<PointT>::Ptr input) 
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);

    
    float theta = -M_PI/15; // The angle of rotation in radians

    // Define a translation of 2.5 meters on the x axis.
    transform.translation() << 0.0, -1.5, 0.0;

    // The same rotation matrix as before; theta radians around Z axis
    transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    printf ("\nTransformation matrix using an Affine3f:\n");
    std::cout << transform.matrix() << std::endl;

    // Executing the transformation
    pcl::transformPointCloud (*input, *output, transform);
    
    
    return(output);    
}



// ########## KMEANS CLUSTERING COMPUTATION ##########
std::vector<pcl::PointIndices> cluster_kmeans (pcl::PointCloud<PointT>::Ptr input, std::vector<pcl::Kmeans::Point> &centroids)
{
     std::size_t num_clusters = centroids.size();
    // Kmeans computation
    pcl::Kmeans kmeans(static_cast<int> (input->points.size()), 3); 
    
    
    kmeans.setClusterSize(num_clusters);
    kmeans.setClusterCentroids(centroids);
    std::vector<pcl::Kmeans::Point> data(input->points.size());
    for(std::size_t i=0; i<input->points.size(); ++i) 
    {
        data[i].push_back(input->points[i].x);
        data[i].push_back(input->points[i].y);
        data[i].push_back(input->points[i].z);
    }
    kmeans.setInputData (data);
    
    kmeans.kMeans();
    
    pcl::Kmeans::Centroids ret_centroids = kmeans.get_centroids(); 
       
     // DEBUG
    /* std::cout << "centroid count: " << ret_centroids.size() << std::endl;
        for (int i = 0; i<ret_centroids.size(); i++)
        {
            std::cout << i << "_cent output: x: " << ret_centroids[i][0] << " ,";
            std::cout << "y: " << ret_centroids[i][1] << " ,";
            std::cout << "z: " << ret_centroids[i][2] << std::endl;
        }
    */

    pcl::Kmeans::ClustersToPoints clusters_to_points;
    clusters_to_points = kmeans.get_clusters_to_points();
    
    std::vector<pcl::PointIndices> output_clusters_indices (num_clusters);
    for (size_t i = 0; i < num_clusters; i++)
    {
        if(!clusters_to_points[i].empty())
        {
            std::set<pcl::Kmeans::PointId>::iterator it = clusters_to_points[i].begin();
 
            // Iterate till the end of set
            while (it != clusters_to_points[i].end())
            {
                output_clusters_indices[i].indices.push_back(static_cast<int>(*it));
                it++;
            }
        }
    }  
        
    return(output_clusters_indices);
}



// ########## KMEANS CENTROIDS COMPUTATION ##########
std::vector<pcl::Kmeans::Point> kmeans_centroids_computation(pcl::PointCloud<PointT>::Ptr input, std::vector<pcl::PointIndices> &input_clusters_indices)
{
    // centroid computation for kmeans clustering 
    std::size_t num_clusters = input_clusters_indices.size();
    std::vector< pcl::Kmeans::Point > centroids(num_clusters);
    std::vector< pcl::PointIndices >::iterator i_segment;
    int i = 0;
    for (i_segment = input_clusters_indices.begin (); i_segment != input_clusters_indices.end (); i_segment++)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*input, i_segment->indices, centroid);
        centroids[i].push_back (centroid[0]);
        centroids[i].push_back (centroid[1]);
        centroids[i].push_back (centroid[2]);
            
        // DEBUG
        /*std::cout << next_color << "_cent output: x: " << centroids[next_color][0] << " ,";
        std::cout << "y: " << centroids[next_color][1] << " ,";
        std::cout << "z: " << centroids[next_color][2] << std::endl;
        */
        
        i++;
    }
    
    return(centroids);
}



// ########## TRUNKS CLUSTERING ##########
std::vector<pcl::PointIndices> cluster_trunks (pcl::PointCloud<PointT>::Ptr input)
{
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());     // Creating the KdTree object for the search method of the extraction
    std::vector<pcl::PointIndices> output;
    pcl::EuclideanClusterExtraction<PointT> ec;



    tree->setInputCloud (input);

    // Euclidean Cluster Extraction
    ec.setClusterTolerance (0.07);  // 7cm
    ec.setMinClusterSize (100);     // this low threshold could be used to filter out small clusters
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input);
    ec.extract (output);
    
    return(output);
}



// ########## Z FILTERING: TRUNKS ##########
pcl::PointCloud<PointT>::Ptr filter_z (pcl::PointCloud<PointT>::Ptr input)
{
    pcl::PassThrough<PointT> pass;
    pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);

    
    // remove ground points and points above the plants
    pass.setInputCloud (input);
    pass.setFilterFieldName ("z");
    pass.setNegative (false);
    pass.setFilterLimits (-0.7, -0.17);
    pass.filter (*output);
    
    return(output);
}



// ########## GROUND REMOVAL ##########
void remove_ground (pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr output, pcl::ModelCoefficients::Ptr coefficients_plane)
{
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_ground_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::ExtractIndices<PointT> extract;
    pcl::PointCloud<PointT>::Ptr cloud_ground (new pcl::PointCloud<PointT> ());
    pcl::PCDWriter writer;

    
    // Estimate point normals
    // It could worth a try to remove normals computation as ground terrain is not pefectly flat
    ne.setSearchMethod (tree);
    ne.setInputCloud (input);
    ne.setKSearch (50);
    ne.compute (*cloud_ground_normals);
    
    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.17);
    seg.setInputCloud (input);
    seg.setInputNormals (cloud_ground_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    if (inliers_plane->indices.size () == 0)
    { 
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }
    std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;

     // Extract the planar inliers from the input cloud
    extract.setInputCloud (input);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);
    // Write the planar inliers to disk
    extract.filter (*cloud_ground);
    
    std::cout << "PointCloud representing the planar ground component has: " << cloud_ground->points.size () << " data points." << std::endl;
    writer.write ("cloud_ground.pcd", *cloud_ground, false);
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*output);
}



// ########## FILTERING ##########
pcl::PointCloud<PointT>::Ptr filter (pcl::PointCloud<PointT>::Ptr input)
{
    pcl::PassThrough<PointT> pass;
    pcl::IndicesPtr indices (new std::vector<int>);
    pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);
    
    // remove the initial e final points along x axis
    pass.setInputCloud (input);
    pass.setFilterFieldName ("x");
    pass.setNegative (false);
    pass.setFilterLimits (-30.0, -1.0);
    pass.filter (*indices);
    pass.setIndices (indices);
    
    // remove lateral points outside the vineyard row
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-4.0, 2.0);
    pass.filter (*indices);
    pass.setIndices (indices);
    
    // remove the central points of the lidar trace
    pass.setFilterFieldName ("y");
    pass.setNegative (true);
    pass.setFilterLimits (-1.5, 0.0);
    pass.filter (*indices);
    pass.setIndices (indices);

    // remove low intensity points
    pass.setFilterFieldName ("intensity");
    pass.setNegative (false);
    pass.setFilterLimits (600, FLT_MAX);
    pass.filter (*indices);
    //pass.setIndices (indices);
    
    // Create the outlier filtering object
    // this filter could remove lower parts of some trunks so it has been disabled
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (input);
    sor.setIndices (indices);
    sor.setMeanK (100);
    sor.setStddevMulThresh (1.3);
    sor.filter (*output);
    
    return(output);
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud (pcl::PointCloud<PointT>& input, std::vector <pcl::PointIndices>& clusters)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
    std::size_t num_clusters = clusters.size();
    
    if (!clusters.empty ())
    {
        colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

        srand (static_cast<unsigned int> (time (0)));
        std::vector<unsigned char> colors;
        for (size_t i_segment = 0; i_segment < num_clusters; i_segment++)
        {
            colors.push_back (static_cast<unsigned char> (rand () % 256));
            colors.push_back (static_cast<unsigned char> (rand () % 256));
            colors.push_back (static_cast<unsigned char> (rand () % 256));
        }
        
        colored_cloud->width = input.width;
        colored_cloud->height = input.height;
        colored_cloud->is_dense = input.is_dense;
        for (size_t i_point = 0; i_point < input.points.size (); i_point++)
        {
            pcl::PointXYZRGB point;
            point.x = *(input.points[i_point].data);
            point.y = *(input.points[i_point].data + 1);
            point.z = *(input.points[i_point].data + 2);
            point.r = 255;
            point.g = 0;
            point.b = 0;
            colored_cloud->points.push_back (point);
        }
        
        std::vector< pcl::PointIndices >::iterator i_segment;
        int next_color = 0;
        for (i_segment = clusters.begin (); i_segment != clusters.end (); i_segment++)
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
            
            next_color++;       
        }
    }

    return (colored_cloud);
}
