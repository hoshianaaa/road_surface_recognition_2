#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <road_surface_recognition_2/cloud_common.h>


class CloudLoadingHandler
{
public:
    CloudLoadingHandler(ros::NodeHandle &nh) :
        //loader_(CloudLoader(nh, "cloud_raw", "table_scene_lms400.pcd"))
        //loader_(CloudLoader(nh, "cloud_raw", "test_pcd.pcd"))
        loader_(CloudLoader(nh, "cloud_raw", "autoware-190620.pcd"))
    {}

    void operate()
    {
        loader_.load();
        loader_.convertPCLtoROS();
    }

    void publish()
    {
        loader_.publish();
    }

private:
    CloudLoader loader_;
};


main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_cloud_from_pcd");
    ros::NodeHandle nh;

    CloudLoadingHandler handler(nh);
    handler.operate();

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        handler.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

