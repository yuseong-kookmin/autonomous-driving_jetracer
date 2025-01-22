/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	sensor_msgs::PointCloud msgCloud;
	laser_geometry::LaserProjection projector_;
	projector_.projectLaser(*scan, msgCloud);

	pcl::PointCloud<pcl::PointXYZ> pclCloud;
	pcl::PointCloud<pcl::PointXYZ> tmpCloud;
	pclCloud.width = msgCloud.points.size();
	pclCloud.height = 1;
	pclCloud.points.resize(pclCloud.width * pclCloud.height);
	for(int i=0; i<msgCloud.points.size(); i++){
		pclCloud.points[i].x = msgCloud.points[i].x;
		pclCloud.points[i].y = msgCloud.points[i].y;
		pclCloud.points[i].z = 0;
		if(pclCloud.points[i].x < 0.35 && pclCloud.points[i].x >-0.35){
		if(pclCloud.points[i].y >0.1 && pclCloud.points[i].y < 0.6)
			tmpCloud.push_back(pclCloud.points[i]);
}
}
	
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(tmpCloud, output);
	std_msgs::Header h = scan->header;
	output.header.frame_id = "/map";
	output.header.stamp = h.stamp;
	pub.publish(output);
	
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/Laser2PointCloud",1);

    ros::spin();

    return 0;
}
