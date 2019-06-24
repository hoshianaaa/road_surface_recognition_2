#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "sensor_msgs/point_cloud_conversion.h"

#include <time.h>
#include <assert.h>

#include <road_surface_recognition_2/map.h>

#include <iostream>
#include <string>
#include <math.h>


map_t *map_alloc(void)
{
    map_t *map;
    
    map = (map_t*) malloc(sizeof(map_t));

    map->origin_x = 0;
    map->origin_y = 0;

    map->size_x = 0;
    map->size_y = 0;
    map->scale = 0;
    
    map->cells = (map_cell_t*) NULL;

    return map;
}


void map_free(map_t *map)
{
    free(map->cells);
    free(map);
    return;
}


void map_update_cell(map_t *map, double gx, double gy, double data)
{
    int mi = MAP_GXWX(map, gx), mj = MAP_GYWY(map, gy);

    if (!MAP_VALID(map, mi, mj))
        return;
/*
    ROS_INFO("mi: %i  mj: %i  valid: %i",mi,mj,MAP_VALID(map, mi, mj));
	mi:1900~2011  mj:1900~2016
*/
    //ROS_INFO("map->size_x:%i",map->size_x);//4672


    map->cells[MAP_INDEX(map,mi,mj)].visit = map->cells[MAP_INDEX(map,mi,mj)].visit + 1;
    map->cells[MAP_INDEX(map,mi,mj)].average = 
        ((map->cells[MAP_INDEX(map,mi,mj)].average*(map->cells[MAP_INDEX(map,mi,mj)].visit-1))  + data)
        / map->cells[MAP_INDEX(map,mi,mj)].visit;
    //ROS_INFO("index:%i",MAP_INDEX(map,mi,mj));//9420763~10000000
    //ROS_INFO("map->cells[MAP_INDEX(map,mi,mj)].visit :%d",map->cells[MAP_INDEX(map,mi,mj)].visit ); 10~16000
    //ROS_INFO("map->cells[MAP_INDEX(map,mi,mj)].average:%f",map->cells[MAP_INDEX(map,mi,mj)].average ); 0.1 ~ 1


    if(map->cells[MAP_INDEX(map,mi,mj)].average >= 30 )
        map->cells[MAP_INDEX(map,mi,mj)].average = 100;
    else
        map->cells[MAP_INDEX(map,mi,mj)].average = 0.1;

    //if(map->cells[MAP_INDEX(map,mi,mj)].average != 100)
        //map->cells[MAP_INDEX(map,mi,mj)].average = 0.1;
}



class ReflectionIntensityMappingNode
{
    public:
        ReflectionIntensityMappingNode();

        void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2);
        void requestMap();
        void map_update_cell2(double gx, double gy, double value);
        void makingOccupancyGridMap();
        map_t* convertMap(const nav_msgs::OccupancyGrid& map_msg );
        bool pubmapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

        /* Variable for map_update_cell2 function. */
        int    *data1;
        int    *data2;
        int    map_H, map_W;
        double map_R;
        double map_CX, map_CY;


    private:
        ros::Subscriber scan_sub;
        ros::Subscriber point_cloud_sub;
	ros::Publisher map_cloud_pub;
        ros::Publisher occupancyGrid_pub;
        ros::ServiceServer pub_server_;

        tf::TransformListener tf_;
        sensor_msgs::PointCloud2 cp_in_cloud;

        std::string global_frame_id_, point_cloud_topic_name_;

        map_t *map_;

        ros::NodeHandle private_nh_;
};

ReflectionIntensityMappingNode::ReflectionIntensityMappingNode()
{
    ros::NodeHandle nh;

    std::string point_cloud_topic_name;
    //private_nh_.param("point_cloud_topic_name", point_cloud_topic_name_, std::string("/cloud_raw"));
    private_nh_.param("point_cloud_topic_name", point_cloud_topic_name_, std::string("/cloud_filtered"));
    private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
    map_cloud_pub     = nh.advertise<sensor_msgs::PointCloud>("map_cloud", 2, true);
    point_cloud_sub   = nh.subscribe(point_cloud_topic_name_, 10, &ReflectionIntensityMappingNode::pointcloudCallback, this);
    occupancyGrid_pub = nh.advertise<nav_msgs::OccupancyGrid>("lawnOccupancyGrid", 2, true);
    pub_server_       = nh.advertiseService("up_map", &ReflectionIntensityMappingNode::pubmapCallback, this);

    requestMap();
}


void ReflectionIntensityMappingNode::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    ROS_INFO("Function in pointcloudCallback");

    /* Convert PointCloud2 to pcl data */
    cp_in_cloud = *point_cloud2;
    cp_in_cloud.fields[3].name = "intensity";
    pcl::fromROSMsg(cp_in_cloud, *pcl_point_cloud);

    /* Run an endpoint update until you run 'rosservice call /up_map'. */
    for(int i=0; i<pcl_point_cloud->points.size(); i++){
        //ROS_INFO("map_update_cell");
        //map_update_cell(map_, pcl_point_cloud->points[i].x, pcl_point_cloud->points[i].y , pcl_point_cloud->points[i].intensity);
	//回転行列
	double theta = -0.1;
	double offset_x = 0;
	double offset_y = -0.7;
        map_update_cell(map_, (pcl_point_cloud->points[i].x * cos(theta) + pcl_point_cloud->points[i].y * sin(theta)) + offset_x  , (pcl_point_cloud->points[i].y * cos(theta) - pcl_point_cloud->points[i].x * sin(theta)) + offset_y , pcl_point_cloud->points[i].intensity);
    	//ROS_INFO("map_->cells[i]:%f  map_->size_x:%d", map_->cells[i], map_->size_x);
	
    }
}

/* When rosservice call /up_map run, this function is called. */
bool ReflectionIntensityMappingNode::pubmapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    sensor_msgs::PointCloud map_cloud;
    sensor_msgs::ChannelFloat32 channel;
    sensor_msgs::PointCloud2 cloud2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr occupancy_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    channel.name = std::string("intensity");
    channel.values.shrink_to_fit();
    map_cloud.channels.push_back(channel);

    /* Make a grid pcl data by converting occupancy grid map coordinate. */
    for(int i=0;i<map_->size_x;i++) {
        ROS_INFO("Convert map data to pcd data");
        for(int j=0;j<map_->size_y;j++) {
            if (MAP_VALID(map_, i, j)) {
		//ROS_INFO("map_->cells[MAP_INDEX].av:%f",map_->cells[MAP_INDEX(map_, i, j)].average);//0.000
                if (map_->cells[MAP_INDEX(map_, i, j)].average > 0.0) {
		    //ROS_INFO("map_->size_x:%i",map_->size_x);// 4672
		    //ROS_INFO("MAP_INDEX:%i",MAP_INDEX(map_, 10, 5));// 23370
		    //ROS_INFO("map_->cells[MAP_INDEX].av:%f",map_->cells[MAP_INDEX(map_, i, j)].average);//0.1 or 100.0
                    geometry_msgs::Point32 point;
                    point.x = MAP_WXGX(map_, i);
		    //ROS_INFO("map_wxgx:%f",MAP_WXGX(map_,i));//3.3 ~ 70.55
		    //ROS_INFO("map_wxgx:%f",MAP_WXGX(map_,100));//-95.00
		    //ROS_INFO("map_wxgx:%f",MAP_WXGX(map_,0));//-100
		    //ROS_INFO("map_wxgx:%f",MAP_WXGX(map_,2000));//0
		    //ROS_INFO("map_wxgx:%f",MAP_WXGX(map_,4000));//100
		    //ROS_INFO("map_wxgx:%f",MAP_WXGX(map_,40000));//1900
	 	    //ROS_INFO("map->scale:%f",map_->scale);//0.05000
		    //ROS_INFO("map->size_x:%f",map_->size_x);//0.05000
		    //ROS_INFO("map->size_y:%f",map_->size_y);//0.05000
		    //ROS_INFO("map->origin_x:%f",map_->origin_x);//16.800002
                    point.y = MAP_WXGX(map_, j);
                    map_cloud.points.push_back(point);
                    map_cloud.channels.at(map_cloud.channels.size()-1).values.push_back(map_->cells[MAP_INDEX(map_, i, j)].average);
		    //ROS_INFO("map_->cells[MAP_INDEX(map_,i,j)].average:%f",map_->cells[MAP_INDEX(map_,i,j)].average);// 0.1  100.00
                }
            }
        }
    }

    map_cloud.header.frame_id = global_frame_id_;
    map_cloud.header.stamp = ros::Time::now();
    convertPointCloudToPointCloud2(map_cloud, cloud2);
    pcl::fromROSMsg(cloud2, *pcl_cloud);
    
    // Delete except data being intensity 100.
    for(int i=0; i<pcl_cloud->points.size(); i++){
        if(pcl_cloud->points[i].intensity != 100){
            pcl_cloud->points[i].x = 0;
            pcl_cloud->points[i].y = 0;
            pcl_cloud->points[i].z = 0;
            pcl_cloud->points[i].intensity = 0;
        }
    }

    // Save raw point cloud data.
    pcl::io::savePCDFileASCII ("/home/hoshina/test_raw.pcd", *pcl_cloud);
    ROS_INFO("Saving a raw pcd file Succeeded\n");
    ROS_INFO("Please wait because a raw pcd file is filtered now ...");


    /* Run an Outlier filter */
    cloud_filtered = pcl_cloud;
/*
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (pcl_cloud);
    sor.setMeanK (100);
    sor.setStddevMulThresh (2.0);
    sor.filter (*cloud_filtered);
*/
    // Save filtered point cloud data.
    pcl::io::savePCDFileASCII ("/home/hoshina/test_filtered.pcd", *cloud_filtered);
    ROS_INFO("Saving a filtered pcd file Succeeded");

    map_cloud_pub.publish(map_cloud);
    //ROS_INFO("Published a PointCloud data of which topic name is map_cloud");

    /* Run endpoint update again because grid pcl data was filtered */
    for(int i=0; i<cloud_filtered->points.size(); i++){
        //ROS_INFO("map_update_cell2");
        map_update_cell2(cloud_filtered->points[i].x, cloud_filtered->points[i].y, cloud_filtered->points[i].intensity);
    }

    makingOccupancyGridMap();

    return true;
}

map_t* ReflectionIntensityMappingNode::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
    map_t* map = map_alloc();

    map->size_x   = map_W  = map_msg.info.width;
    //ROS_INFO("map_W:%i[cells]",map_W);
    map->size_y   = map_H  = map_msg.info.height;
    //ROS_INFO("map_H:%i[cells]",map_H);
    map->scale    = map_R  = map_msg.info.resolution*1.0;
    //ROS_INFO("map_R:%f[m/cells]",map_R);
    map->origin_x = map_CX = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
    //ROS_INFO("map_msg.info.origin.position.x :%f[m]",map_msg.info.origin.position.x);
    //ROS_INFO("map_CX:%f",map_CX);
    map->origin_y = map_CY = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
    //ROS_INFO("map_msg.info.origin.position.y :%f[m]",map_msg.info.origin.position.y);
    //ROS_INFO("map_CY:%f",map_CY);
    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
    //ROS_INFO("map_cells:%f",map->cells);

    data2 = new int[map_W*map_H];
    data1 = new int[map_W*map_H];

    for(int i = 0; i < map_msg.data.size(); i++)
        data1[i] = map_msg.data[i];

    return map;
}


void ReflectionIntensityMappingNode::requestMap()
{
    nav_msgs::GetMap::Request  req;
    nav_msgs::GetMap::Response resp;

    ROS_INFO("Requesting the map...");
    while(!ros::service::call("static_map", req, resp))
    {
        ROS_WARN("Request for map failed; trying again...");
        ros::Duration d(0.5);
        d.sleep();
    }

    /* Import map_server data */
    map_ = convertMap(resp.map);
}


void ReflectionIntensityMappingNode::makingOccupancyGridMap()
{
    nav_msgs::OccupancyGrid lawn_occupancy;

    lawn_occupancy.header.frame_id = global_frame_id_;
    lawn_occupancy.info.width = map_W;
    lawn_occupancy.info.height = map_H;
    lawn_occupancy.info.resolution = map_R;
    lawn_occupancy.info.origin.position.x = map_CX - (map_W / 2) * map_R;
    lawn_occupancy.info.origin.position.y = map_CY - (map_H / 2) * map_R;
    lawn_occupancy.info.origin.position.z = 0.0;

    /* Make occupancy grid map which has lawn data. 
     * This map is made from grid pcl data and a published occupancy grid map from topic named map_server. */
    for(int i = 0; i < map_W*map_H; i++){
        lawn_occupancy.data.push_back((int)data1[i]);
        if(lawn_occupancy.data[i] != 100 && (int)data2[i] == 100)
            lawn_occupancy.data[i] = (int)data2[i];
    }
    
    // If you want to save occupancy grid map that has lawn place data regarded as occupation, 
    // Please run a following command. 
    // 'rosrun map_server map_saver -f mapfilename map:=lawnOccupancyGrid' 
    occupancyGrid_pub.publish(lawn_occupancy);
    ROS_INFO("Published an OccupancyGrid data of which topic name is occupancyGrid");

    return ;
}

// Endpoint update function 2.
void ReflectionIntensityMappingNode::map_update_cell2(double gx, double gy, double value)
{
    double x = gx - map_CX;
    double y = gy - map_CY;
    int ix = (int)(x/map_R + map_W/2 + 0.5);
    int iy = (int)(y/map_R + map_H/2 + 0.5);
    int i = map_W * iy + ix;

    data2[abs(i)] = value;

    return ;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "reflection_intensity_mapping");
    ReflectionIntensityMappingNode ri;

    ros::spin();

    return 0;
}

