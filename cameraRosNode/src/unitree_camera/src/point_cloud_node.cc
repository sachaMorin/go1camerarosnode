#include <iostream>
#include <signal.h>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <UnitreeCameraSDK.hpp>

bool killSignalFlag = false;
void ctrl_c_handler(int s){
    killSignalFlag = true;
    return ;
}

int main(int argc, char **argv)
{
    std::string node_name;
    if(argc < 3){
        printf("Tips: if use rosrun, execute: rosrun pkgname node_executable_bin node_name _params:=value ...\n");
        node_name = std::string("unitree_camera_node");
    }else{
        node_name = std::string(argv[2]);
    }
    double offsetTime = static_cast<double>(std::atof(argv[3]));

    ros::init(argc, argv, node_name);
    ros::Publisher topic_publisher;
    ros::Publisher range_publisher;

    ros::NodeHandle node_handler("~");
    
    std::string config_file;
    node_handler.getParam("config_file", config_file);
    std::string run_config_file;
    node_handler.param<std::string>("run_config_file", run_config_file, "");
    
    std::string cameraConfig;
    if(config_file.size() != 0){
        cameraConfig = config_file;
    }else if(run_config_file.size() != 0){
        cameraConfig = run_config_file;
    }
    
    std::cout << "Camera Config File:" << cameraConfig << std::endl;
    
    UnitreeCamera cam(cameraConfig);
    if(!cam.isOpened())
        return -1;
    int camPosNum = cam.getPosNumber();
    std::string frameIdName = "trunk";
    std::string pointCloudName = "point_cloud_";
    std::string rangeName = "range_visual_";
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ctrl_c_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    float theta, tanTheta,tanBk;
    cv::Mat vectorInCamFrame = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat vectorInBodyFrame = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat translationCamToBody, rotationCamToBody;
    cam.startCapture(false, true);
    if (camPosNum!=2)
{
    cam.startStereoCompute();




    


    switch(camPosNum){
    case 1:{pointCloudName += "face"; rangeName += "face";}break;
    case 2:{pointCloudName += "chin";}break;
    case 3:{pointCloudName += "left"; rangeName += "left";}break;
    case 4:{pointCloudName += "right"; rangeName += "right";}break;
    case 5:{pointCloudName += "rearDown";}break;
    default:{;}
    }
    
    std::cout << "Camera PositionNumber -> " << camPosNum << " Point Cloud Name ->" << pointCloudName << std::endl;
    



    switch(camPosNum){
    case 1:{
        theta = 2.0 * M_PI / 3.0 + M_PI / 18;
        tanTheta = std::tan((M_PI - theta) / 2);
        translationCamToBody = (cv::Mat_<float>(3,1) << 0.28, 0.010, 0.0536);
        rotationCamToBody = getTranslationMatrix('x', M_PI) *  getTranslationMatrix('y', M_PI / 2.0) * getTranslationMatrix('z', -M_PI / 2);
    }break;
    case 2:{
        theta = 2.0 * M_PI / 3.0 + M_PI / 18;
        tanBk = std::tan(M_PI / 4 - M_PI / 16);
        tanTheta = std::tan((M_PI - theta) / 2);
        translationCamToBody = (cv::Mat_<float>(3,1) << 0.287, 0.010, -0.0046);
        rotationCamToBody = getTranslationMatrix('z', M_PI)  * getTranslationMatrix('y', -M_PI / 48) * getTranslationMatrix('y', M_PI) * getTranslationMatrix('z', -M_PI / 2);
    }break;
    case 3:{
        tanTheta = std::tan(M_PI / 2.5);
        translationCamToBody = (cv::Mat_<float>(3,1) << -0.0135, 0.080, 0.0136);
        rotationCamToBody = getTranslationMatrix('y', M_PI) * getTranslationMatrix('x', -(M_PI / 2 -  M_PI / 16));
    }break;
    case 4:{
        theta = M_PI * 5.0 / 6.0 - M_PI * 2 / 9;
        tanTheta = std::tan((M_PI - theta) / 2.0);
        translationCamToBody = (cv::Mat_<float>(3,1) << -0.0105, -0.080, 0.0236);
        rotationCamToBody = getTranslationMatrix('y', M_PI) * getTranslationMatrix('x', (M_PI / 2.0 - M_PI / 16.0)) * getTranslationMatrix('z', M_PI);
    }break;
    case 5:{
        theta = M_PI * 2.0 / 3.0;
        tanTheta = std::tan((M_PI - theta) / 2.0);
        translationCamToBody = (cv::Mat_<float>(3,1) << -0.0505, 0, -0.0055);
        rotationCamToBody = getTranslationMatrix('x', M_PI) * getTranslationMatrix('z', -M_PI / 2);
    }break;
    default:{;}
    }


    topic_publisher = node_handler.advertise<sensor_msgs::PointCloud2>(pointCloudName, 1);
    if (camPosNum==1||camPosNum ==3||camPosNum==4)
        range_publisher = node_handler.advertise<sensor_msgs::Range>(rangeName, 1);
}
    while(ros::ok() && cam.isOpened()){
        
        if(killSignalFlag){
            break;
        }

        std::chrono::microseconds timeStamp;
        std::vector<PCLType> curPCL;
        if(!cam.getPointCloud(curPCL, timeStamp)){
            usleep(1000);
            continue;
        }
	if(camPosNum==2)
{
	sleep(0.005);
	continue;
}
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<PCLType>::iterator iter;
        for(iter = curPCL.begin(); iter != curPCL.end(); iter++){
            pcl::PointXYZRGB point;
            vectorInCamFrame.at<float>(0,0) = (*iter).pts(0);
            vectorInCamFrame.at<float>(1,0) = (*iter).pts(1);
            vectorInCamFrame.at<float>(2,0) = (*iter).pts(2);

            //Pb = Rbc * Pc + Tbc;
            vectorInBodyFrame = rotationCamToBody * vectorInCamFrame;
            point.x = (vectorInBodyFrame.at<float>(0,0) + translationCamToBody.at<float>(0,0));
            point.y = (vectorInBodyFrame.at<float>(1,0) + translationCamToBody.at<float>(1,0));
            point.z = (vectorInBodyFrame.at<float>(2,0) + translationCamToBody.at<float>(2,0));

            float px = vectorInBodyFrame.at<float>(0,0);
            float py = vectorInBodyFrame.at<float>(1,0);
            float pz = vectorInBodyFrame.at<float>(2,0);

            if(camPosNum == 1){
                if(point.z > 0.15)
                    continue;
                if(px<0.05)
                    continue;
//                if(std::abs(px / py) < tanTheta)
//                    continue;
                if(std::abs(px / py) < std::tan(M_PI/180.0*40.0)) // range of 100
                    continue;
                if (!(py < 0.5 && py > -0.5))
                    continue;
            }else if(camPosNum == 2){
                float tanPtx = std::abs(pz / px);
                float tanPty = std::abs(pz / py);
                if( tanPtx < tanTheta || tanPty < tanTheta || (tanPtx < tanBk && px < 0 ))
                    continue;
            }else if(camPosNum == 3){ // left
                if( point.z > 0.15 || std::abs(py/px) < std::tan(M_PI/180.0*40.0))//range of 100
                    continue;
//                if( point.z > -0.26 && point.y < 0.25 && (std::abs(py / px) < tanTheta || std::abs(py / pz) < tanTheta))
//                    continue;
                if(!(py > 0.1 && px < 0.35 && px > -0.35))
                    continue;

            }else if(camPosNum == 4){ // right
                if( point.z > 0.15 || std::abs(py / px) < std::tan(M_PI/180.0*40.0))//range of 100
                    continue;
                if(!(py <-0.1 && px < 0.35 && px > -0.35))
                    continue;
            }else if(camPosNum == 5){
                if(std::abs(pz / py) < tanTheta || std::abs(pz / px) < tanTheta)
                    continue;
                if(!(point.z < -0.05 && point.y >= -0.35 && point.y <= 0.35 && point.x < 0.80 && point.x > -0.8))
                    continue;
            }

            point.r = (*iter).clr(2);point.g = (*iter).clr(1);point.b = (*iter).clr(0);
            cloud->points.push_back(point);
        }
        
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(30);
        if(camPosNum == 1)
            sor.setStddevMulThresh(1.0f);
        else
            sor.setStddevMulThresh(1.0f);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        sor.filter(*cloud_filtered);


        sensor_msgs::PointCloud2 cloud_publish_msg;
        pcl::toROSMsg(*cloud_filtered, cloud_publish_msg);
        cloud_publish_msg.header.frame_id = "trunk";
        cloud_publish_msg.header.stamp =  ros::Time(timeStamp.count() / 1000000.0 + offsetTime);
        topic_publisher.publish(cloud_publish_msg);
        //	printf("%s\n", argv[4]);
        //	printf("offset : %f\n", offsetTime);
        //	printf("stamp: %lf\n", timeStamp.count() / 1000000 + std::atof(argv[4]));

        // publish ranges for front, left and right cameras
        sensor_msgs::Range range;

        switch(camPosNum){
        case 1:
        {
            float dmin = 2.0;
            for (size_t i = 0; i<cloud_filtered->size(); i++)
            {
                if (std::fabs(cloud_filtered->at(i).z + 0.3) > 0.2 && cloud_filtered->at(i).x < dmin)
                    dmin = cloud_filtered->at(i).x;
            }

            range.range = dmin - 0.2747;
            range.field_of_view = 90.0/180.0*M_PI;
            range.min_range = 0.05;
            range.max_range = 2.0;
            range.radiation_type = sensor_msgs::Range::ULTRASOUND;
            range.header.frame_id = "camera_face";
            range.header.stamp = ros::Time(timeStamp.count() / 1000000.0 + offsetTime);
            range_publisher.publish(range);

        }break;

        case 3:
        {
            float dmin = 2.0;
            for (size_t i = 0; i<cloud_filtered->size(); i++)
            {
                if (std::fabs(cloud_filtered->at(i).z + 0.3) > 0.2 && std::fabs(cloud_filtered->at(i).y) < dmin)
                    dmin = std::fabs(cloud_filtered->at(i).y);
            }

            range.range = dmin - 0.0826;
            range.field_of_view = 90.0/180.0*M_PI;
            range.min_range = 0.05;
            range.max_range = 2.0;
            range.radiation_type = sensor_msgs::Range::ULTRASOUND;
            range.header.frame_id = "camera_left";
            range.header.stamp = ros::Time(timeStamp.count() / 1000000.0 + offsetTime);
            range_publisher.publish(range);

        }break;
        case 4:
        {
            float dmin = 2.0;
            for (size_t i = 0; i<cloud_filtered->size(); i++)
            {
                if (std::fabs(cloud_filtered->at(i).z + 0.3) > 0.15 && std::fabs(cloud_filtered->at(i).y) < dmin)
                    dmin = std::fabs(cloud_filtered->at(i).y);
            }

            range.range = dmin - 0.0826;
            range.field_of_view = 90.0/180.0*M_PI;
            range.min_range = 0.05;
            range.max_range = 2.0;
            range.radiation_type = sensor_msgs::Range::ULTRASOUND;
            range.header.frame_id = "camera_right";
            range.header.stamp = ros::Time(timeStamp.count() / 1000000.0 + offsetTime);
            range_publisher.publish(range);

        }break;

        }
    }
    
    cam.stopStereoCompute();
    usleep(500000);
    cam.stopCapture();

    return 0;
}
