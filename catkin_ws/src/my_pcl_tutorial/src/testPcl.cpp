#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <fstream>
// #include <boost>

using namespace std;

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud.makeShared ());
    seg.segment (inliers, coefficients); 
    
    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub.publish (ros_coefficients);
}

main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");

    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/pcl/sub", 10, cloud_cb);

    // Create a ROS publisher for the output model coefficients
    // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
    pub = nh.advertise< sensor_msgs::PointCloud2 > ("pcl/pub", 10, true);

    /*    
    // Spin
    ros::spin ();
    */

    std::string path = "/home/rxth/rakshith/data/work/asu/dreams/treeMapping/dataset/4_LidarTreePoinCloudData/";
    std::string fileName = "GUY01_000.txt";
    path += fileName;

    float x, y, z;

    // pcl::PointCloud<pcl::PointXYZ> cloud; //Convert to pointer type
    // wrong since it returns a pointer. But ::Ptr type is already a pointer of its own kind. So just initialize it as an arg to the datatype
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = new pcl::PointCloud<pcl::PointXYZ>(); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point;

    int ctr = 0;
    int tmp;
    std::ifstream fileIn(path);
    while(fileIn >> x >> y >> z)
    {
        if(ctr >= 3)
        {
            break;
        }

        // if(ctr % 10 == 0)
        // {
            cout << "ctr: " << ctr << endl;
        // }

        cout << "xyz: " << x << ", " << y << ",  " << z << endl;
        point.x = x;
        point.y = y;
        point.z = z;
        cloudPcl->push_back(point);

        ctr += 1;
    }

    sensor_msgs::PointCloud2 cloudRos;
    pcl::toROSMsg(*(cloudPcl), cloudRos);

    pub.publish(cloudRos);


    while(1)
    {}

    // boost::shared_ptr viewer (new pcl::visualization::PCLVisualizer ("3D    Viewer"));
    // viewer->setBackgroundColor (0, 0, 0);
    // viewer->addCoordinateSystem (1.0);
    // viewer->initCameraParameters ();

    cout << "Done" << endl;
    return 0;
}
