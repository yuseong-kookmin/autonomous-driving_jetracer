#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>

ros::Publisher pub1;
ros::Publisher pub2;
int cnt =0;
using namespace std;
int m_s=0;
float calculateDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){
	float dx = p1.x -p2.x;
	float dy = p1.y - p2.y;
	return sqrt(dx*dx+dy*dy);
}
bool comparePoints(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){
	pcl::PointXYZ origin;
	origin.x = 0.0;
	origin.y = 0.0;
	float distance1 = calculateDistance(p1,origin);
	float distance2 = calculateDistance(p2,origin);
	return distance1 < distance2;
}

void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg){

    
    ros::param::get("/m_s", m_s);
    if(m_s == 2){
    pcl::PointCloud<pcl::PointXYZ> inputCloud;

    pcl::fromROSMsg(*msg, inputCloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud (inputCloud.makeShared());

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1); // set distance threshold = 1.5m
    ec.setMinClusterSize (5); // set Minimum Cluster Size
    ec.setMaxClusterSize (2500); // set Maximum Cluster Size
    ec.setSearchMethod (kdtree);
    ec.setInputCloud (inputCloud.makeShared());
    ec.extract (clusterIndices);

    int clusterN = 1;
    
    pcl::PointCloud<pcl::PointXYZ> tmpCloud;
    sensor_msgs::PointCloud2 output;
    for (const pcl::PointIndices& cluster : clusterIndices){
		float minDistance=std::numeric_limits<float>::max();
		pcl::PointXYZ closestPoint;

		for(const int index : cluster.indices){
			const pcl::PointXYZ& point = inputCloud[index];
			float distance = std::sqrt(point.x*point.x+point.y*point.y);
		if (distance<minDistance){
			minDistance=distance;
			closestPoint=point;
		}

		
		}
		
		tmpCloud.push_back(closestPoint);
		
	}
    sort(tmpCloud.begin(),tmpCloud.end(),comparePoints);
    //tmpCloud = clustered point
    pcl::toROSMsg(tmpCloud,output);
    
    
    if(tmpCloud.size() !=0){
	cnt = cnt+1;
	if(cnt>=10)
		cnt = 10;}
    else{
	cnt = cnt-1;
	if(cnt<=0)
		cnt = 0;}
    if(cnt>=1)
        ros::param::set("/is_dynamic",1);
    else
	ros::param::set("/is_dynamic",0);

    cout <<cnt<<endl;
    





    output.header.frame_id="/map";
    pub1.publish(output);
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "clustering");
    ros::NodeHandle nh;

    // << Subscribe Topic >>
    // topic name : /passPC
    // topic type : sensor_msgs::PointCloud2
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/Laser2PointCloud", 1, callbackFcn);

    // << Publish Topic >>
    // topic name : /cluster1PC
    // topic type : sensor_msgs::PointCloud2
    pub1 = nh.advertise<sensor_msgs::PointCloud2>("/cluster1PC", 1);
    
    
    // << Publish Topic >>
    // topic name : /cluster2PC
    // topic type : sensor_msgs::PointCloud2
    //pub2 = nh.advertise<sensor_msgs::PointCloud2>("/cluster2PC", 1);
    ros::spin();
    return 0;
}
