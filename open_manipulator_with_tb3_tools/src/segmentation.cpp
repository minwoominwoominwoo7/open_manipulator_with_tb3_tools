#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

ros::Publisher pub;
ros::Publisher pub_pcl_object;

typedef pcl::PointXYZ PointT;
//std_msgs::Bool use_pcl.data = false;
bool use_pcl = false;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{ 
  if (use_pcl == false)
  {
    return ;
  }
  ROS_INFO("cloud_cb"); 
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud); 
 
  //std::cout << "Point Cloud Size" << input->header.frame_id  << std::endl;
  // Data containers used
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud.makeShared());
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.01); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
  float  x_sum = 0; 
  float  y_sum = 0;
  float  z_sum = 0;

  float  min_z = 100; 
  geometry_msgs::PoseStamped object_output; 
  pcl::PointCloud<pcl::PointXYZI> TotalCloud; 
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    x_sum = 0, y_sum = 0, z_sum = 0;    
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
        pcl::PointXYZ pt = cloud_filtered->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(j + 1);
            x_sum = x_sum + pt2.x , y_sum = y_sum + pt2.y , z_sum = z_sum + pt2.z;   
            TotalCloud.push_back(pt2);
    }
    x_sum = x_sum/ it->indices.size(), y_sum = y_sum/ it->indices.size(),z_sum =  z_sum/ it->indices.size();
    //std::cout << "PointCloud after filtering has: " << x_sum <<','<< y_sum <<','<< z_sum << " data points." << std::endl;
    if( min_z > z_sum ) 
    {
      min_z = z_sum ;   
      object_output.pose.position.x = x_sum, object_output.pose.position.y = y_sum, object_output.pose.position.z = z_sum;
      object_output.header.stamp = ros::Time::now();
    }
    printf("%d : %.2f, %.2f, %.2f \n",j, x_sum, y_sum, z_sum);
    //lt.push()
    j++;
  }

  printf("object : %.2f, %.2f, %.2f \n",object_output.pose.position.x, object_output.pose.position.y, object_output.pose.position.z);

    // Convert To ROS data type 
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  
  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = input->header.frame_id ;  
  pub.publish(output);   
  pub_pcl_object.publish(object_output); 
 
  ROS_INFO("published it."); 
}

void
use_pcl_cb (const std_msgs::Bool& input)
{ 
  use_pcl = input.data;
  ROS_INFO("use_pcl_cb %d ", use_pcl); 
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ROS_INFO("main"); 
  use_pcl = true;
  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe ("/om_with_tb3/camera2/depth_registered/points", 1, cloud_cb);
  ros::Subscriber sub_use_pcl = nh.subscribe ("/use_pcl", 1, use_pcl_cb);

  // Create a ROS publisher for the output model coefficients
  // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("pclplaneoutput", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("pclplaneoutput", 1);
  pub_pcl_object = nh.advertise<geometry_msgs::PoseStamped> ("pcl_object_position", 1);
 
  // Spin
  ros::spin ();
}
