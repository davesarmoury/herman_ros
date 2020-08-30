#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class LaserToCloud {
     public:
        LaserToCloud();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

LaserToCloud::LaserToCloud(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &LaserToCloud::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
        tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void LaserToCloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){


  if(!tfListener_.waitForTransform(
    scan->header.frame_id,
    "/world",
    scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
    ros::Duration(1.0))){
       return;
    }

    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("world", *scan, cloud, tfListener_);
    
    point_cloud_publisher_.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_to_cloud");

    LaserToCloud filter;

    ros::spin();

    return 0;
}
