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

using namespace std;


main (int argc, char** argv)
{
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise< sensor_msgs::PointCloud2 > ("pcl/pub", 10, true);

    std::string path = "/home/rxth/rakshith/data/work/asu/dreams/treeMapping/dataset/4_LidarTreePoinCloudData/";
    // std::vector<std::string> fileNames = {"GUY01_000.txt"};
    std::vector<std::string> fileNames = {"GUY01_000.txt", "GUY02_000.txt", "GUY03_000.txt"};
    // std::vector<std::string> fileNames = {"IND01_054.txt", "IND02_067.txt", "IND03_088.txt"};
    // std::vector<std::string> fileNames = {"MDD01_006.txt", "MDD02_013.txt", "MDD03_011.txt"};
    // std::vector<std::string> fileNames = {"GUY01_000.txt", "IND01_054.txt", "MDD01_006.txt"};

    // pcl::PointCloud<pcl::PointXYZ> cloud; //Convert to pointer type
    // wrong since it returns a pointer. But ::Ptr type is already a pointer of its own kind. So just initialize it as an arg to the datatype
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = new pcl::PointCloud<pcl::PointXYZ>(); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point;
    float x, y, z;
    for(int i = 0; i<fileNames.size(); ++i)
    {

        std::string fileName = fileNames[i];
        std::string fullPath = path + fileName;

        int ctr = 0;
        int tmp;
        int maxPoints = 2000000; // Load upto 2 million points per tree
        std::ifstream fileIn(fullPath);
        cout << "Adding Tree: " << fileName << endl;        
        while(fileIn >> x >> y >> z)
        {
            if(ctr >= maxPoints)
            {
                break;
            }

            if( (ctr % (maxPoints/100) ) == 0)
            {
                cout << "ctr: " << ctr << endl;
            }

            // cout << "xyz: " << x << ", " << y << ",  " << z << endl;
            point.x = x;
            point.y = y;
            point.z = z;
            cloudPcl->push_back(point);

            ctr += 1;
        }
    }

    sensor_msgs::PointCloud2 cloudRos;
    pcl::toROSMsg(*(cloudPcl), cloudRos);

    cloudRos.header.frame_id = "origin_tree";
    pub.publish(cloudRos);

    while(ros::ok())
    {
        // just stay alive so that topics are alive
        ros::Duration(1).sleep();
    }

    return 0;
}
