#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <UnitreeCameraSDK.hpp>
#include <unistd.h>

int main(int argc, char *argv[])
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

    ros::NodeHandle node_handler("~");
    image_transport::ImageTransport it(node_handler);
    image_transport::Publisher right_pub = it.advertise("/unitree_camera/head", 1);
    
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

    if (!cam.isOpened())
        exit(EXIT_FAILURE);

    // Start capturing
    cam.startCapture(true, false);

    usleep(500000);

    // Loop and publish images to ROS topics
    while (cam.isOpened() && ros::ok())
    {
        cv::Mat frame;
	std::chrono::microseconds t;
        if (!cam.getRawFrame(frame, t))
        {
            usleep(1000);
            continue;
        }

        // Create ROS Image messages using cv_bridge
        sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        // Publish images
        right_pub.publish(left_msg);
	// right_pub.publish(right_msg);

        ros::spinOnce();

        // Check for ESC key press
        char key = cv::waitKey(10);
        if (key == 27)
            break;
    }

    // Stop capturing
    cam.stopCapture();

    return 0;
}
