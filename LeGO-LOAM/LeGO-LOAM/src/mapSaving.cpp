/*
* Date 			: 2020-05-29 12:57:41
* Author		: Pallav Bhalla (pallav.bhalla@fluxauto.xyz)
* Link			: https://github.com/pallav1299
* Version		: 1.0.0
* Description	: Saving the final PCD, made by stitching the raw XYZIRGBS data.
                  Need the XYZIRGBS pointtype since we are adding the neglected points
                  to pixel_cloud_fusion output with black colour{(R,G,B)=(0,0,0)}. Need 
                  to remove these points after feature association. 
                  
                  Basically, using all features in the raw cloud, the lidar_odometry is 
                  calculated in featureAssociation node. This odometry is used to stitch
                  the raw cloud inspite of the segmented cloud. Hence making the cloud dense.
*/

#include "utility.h"
#include "std_msgs/String.h"

class MapSaving{
private :

    ros::NodeHandle nh;
    ros::Subscriber subRawCloud;    
    ros::Subscriber subLaserOdometry;   
    ros::Subscriber subSave;    

    ros::Publisher pubSubmap;    

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawCloudScan;   
    pcl::PointCloud<PointType>::Ptr rawCloudScanState;   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedRawCloudScan;   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawCloudStitched;   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMap;   
    pcl::PointCloud<pcl::PointXYZRGB> submap;   
    pcl::PointCloud<pcl::PointXYZRGB> savedSubmap;   
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredSubmap;   

    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;   

    nav_msgs::Odometry laserOdometry;   
    tf::StampedTransform laserOdometryTrans;    

    std_msgs::Header cloudHeader;   

    double timeNewRawCloud; 
    double timeNewOdometry; 
    bool newRawCloudScan;   
    bool newOdometryAvailable;    
    
    float leafSize; 
    float x, y, z;  
    int submap_num; 
    float submap_size;  
    float max_submap_size;  

    std::string s1, s2, s3, pcd_filename;   

    float current_x, current_y, current_z;   
    float prev_x, prev_y, prev_z;   


public :

    MapSaving():
        nh("~")
        {   
            subRawCloud = nh.subscribe<sensor_msgs::PointCloud2>("/points_fused", 5, &MapSaving::rawCloudHandler, this);   
            subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/integrated_to_init", 5, &MapSaving::laserOdometryHandler, this);  
            // sbmap.reset(new pcl::PointCloud<PointType>());
            pubSubmap = nh.advertise<sensor_msgs::PointCloud2>("/submap", 5);    

            initializeValues(); 
        }

    void initializeValues()
    {   
        leafSize = 0.1; 
        downSizeFilter.setLeafSize(leafSize, leafSize, leafSize);

        rawCloudScan.reset(new pcl::PointCloud<pcl::PointXYZRGB>());   
        rawCloudScanState.reset(new pcl::PointCloud<PointType>());   
        transformedRawCloudScan.reset(new pcl::PointCloud<pcl::PointXYZRGB>());    
        rawCloudStitched.reset(new pcl::PointCloud<pcl::PointXYZRGB>());    
        globalMap.reset(new pcl::PointCloud<pcl::PointXYZRGB>());   
        // filteredSubmap.reset(new pcl::PointCloud<pcl::PointXYZRGB>());   

        laserOdometryTrans.frame_id_ = "/camera_init";
        laserOdometryTrans.child_frame_id_ = "/laser_odom";

        timeNewRawCloud = 0;    
        timeNewOdometry = 0;    
        newRawCloudScan = false;    
        newOdometryAvailable = false;     
        submap_num = 0; 
        submap_size = 0;    
        max_submap_size = 100;    

        prev_x = 0.0;
        prev_y = 0.0;
        prev_z = 0.0;

        current_x = 0.0;
        current_y = 0.0;
        current_z = 0.0;
    }

    void rawCloudHandler(const sensor_msgs::PointCloud2ConstPtr& rawCloudMsg)
    {
        cloudHeader = rawCloudMsg->header;  
        timeNewRawCloud = rawCloudMsg->header.stamp.toSec();    
        pcl::fromROSMsg(*rawCloudMsg, *rawCloudScanState);   

        //remove all black points
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());    
        pcl::ExtractIndices<PointType> extract; 
        for (int i = 0; i < (*rawCloudScanState).size(); i++)    
        {
            PointType pt;   
            pt = rawCloudScanState->points[i];   
            if (pt.state == 1){   
                inliers->indices.push_back(i);  
            }
        }
        extract.setInputCloud(rawCloudScanState);    
        extract.setIndices(inliers);    
        extract.setNegative(true);  
        extract.filter(*rawCloudScanState);   

        //change pointcloud pointtype to PointXYZRGB
        pcl::PointXYZRGB p;    
        for (int i=0; i<rawCloudScanState->points.size(); i++){   
            //This data is related to the camera frame, which is different w.r.t LIDAR frame
            p.x = rawCloudScanState->points[i].y; 
            p.y = rawCloudScanState->points[i].z; 
            p.z = rawCloudScanState->points[i].x; 
            p.rgb = rawCloudScanState->points[i].rgb;   
            rawCloudScan->push_back(p); 
        }   
        newRawCloudScan = true; 
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometryMsg)
    {
        timeNewOdometry = laserOdometryMsg->header.stamp.toSec();
        laserOdometry = *laserOdometryMsg;

        laserOdometryTrans.stamp_ = laserOdometryMsg->header.stamp;  
        laserOdometryTrans.setRotation(tf::Quaternion(laserOdometry.pose.pose.orientation.x, laserOdometry.pose.pose.orientation.y, laserOdometry.pose.pose.orientation.z, laserOdometry.pose.pose.orientation.w)); 
        laserOdometryTrans.setOrigin(tf::Vector3(laserOdometry.pose.pose.position.x, laserOdometry.pose.pose.position.y, laserOdometry.pose.pose.position.z));  

        current_x = laserOdometry.pose.pose.position.x;
        current_y = laserOdometry.pose.pose.position.y;
        current_z = laserOdometry.pose.pose.position.z;

        newOdometryAvailable = true;
    }

    void mapStitching()
    {   
        pcl_ros::transformPointCloud(*rawCloudScan, *transformedRawCloudScan, laserOdometryTrans);  

        globalMap->points.resize(globalMap->points.size() + transformedRawCloudScan->points.size());    
        *globalMap += *transformedRawCloudScan;  
        *rawCloudStitched += *transformedRawCloudScan;  

        rawCloudScan->clear();  
        transformedRawCloudScan->clear();   
        
    }

    void saveGlobalMapThread()
    {
        ros::Rate rate(10);    
        while (ros::ok()){  
            saveSubmap();   
            rate.sleep();   
        }

        if (globalMap->points.size() != 0)
        {
            submap = *globalMap;    
            submap_num ++;  
            globalMap->clear(); 

            s1 = "/home/pallavbhalla/Documents/LeGO-LOAM/submaps/submap_";  
            s2 = std::to_string(submap_num);    
            s3 = ".pcd";    
            pcd_filename = s1 + s2 + s3;    

            if (pcl::io::savePCDFileBinary(pcd_filename, submap) == -1)  
                {
                    std::cout << "Failed saving " << pcd_filename << "." << std::endl;  
                }
                std::cout << "Saved " << pcd_filename << " (" << submap.size() << " points)" << std::endl;  

                submap.clear(); 
                submap_size = 0.0;  
        }
    }

    void saveSubmap()
    {
        double shift = sqrt(pow(current_x - prev_x, 2.0) + pow(current_y - prev_y, 2.0) + pow(current_z - prev_z, 2.0)); 
        // std::cout << shift << std::endl; 

        submap_size = shift;   
        if (submap_size >= max_submap_size){    
            submap = *globalMap;    
            submap_num ++;  
            submap.height = 1;  
            submap.width = submap.size();    

            globalMap->clear();  

            /*
            Uncomment following two lines to visualize created submaps. This causes RVIZ to crash
            in case of large pointclouds.
            */

            // savedSubmap = submap;   
            // publishGlobalMap(); 

            s1 = "/home/pallavbhalla/Documents/LeGO-LOAM/submaps/submap_";
            s2 = std::to_string(submap_num);
            s3 = ".pcd";
            pcd_filename = s1 + s2 + s3;

            if (submap.size() != 0)
            {
                if (pcl::io::savePCDFileBinary(pcd_filename, submap) == -1)
                {
                    std::cout << "Failed saving " << pcd_filename << "." << std::endl;
                }
                std::cout << "Saved " << pcd_filename << " (" << submap.size() << " points)" << std::endl;

                submap.clear();
                submap_size = 0.0;
            }
            prev_x = current_x;
            prev_y = current_y;
            prev_z = current_z;
        }
    }

    void publishGlobalMap()
    {
        if (pubSubmap.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 laserCloudOutMsg;

            pcl::toROSMsg(savedSubmap, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera_init";
	        pubSubmap.publish(laserCloudOutMsg);
        }
    }

    void run()
    {
        if(newRawCloudScan && newOdometryAvailable && std::abs(timeNewRawCloud - timeNewOdometry)){
            newRawCloudScan = false;
            newOdometryAvailable = false;
        }else{
            return;
        }

        mapStitching();

        publishGlobalMap();
    }
};

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "map_saving");
    MapSaving MS;

    std::thread saveMapThread(&MapSaving::saveGlobalMapThread, &MS);   
    
    ros::Rate rate(200);
    std::cout << "Map MapSaving running..." <<endl;

    while (ros::ok())
    {
        ros::spinOnce();
        MS.run();
        rate.sleep();
    }

    saveMapThread.join();

    return 0;
}