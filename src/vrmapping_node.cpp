#include <vrmapping/vrmapping.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vrmapping_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::GLOG_INFO, "/home/december/logs/");
    google::SetStderrLogging(google::GLOG_INFO);
    google::SetLogFilenameExtension("log_");

    VRMap VRMap_(nh, nh_private);
    ROS_INFO("Hello world, the vrmapping is starting.");
    int spinner_thread;
    nh.param<int>("/vrmapping/vrmapping_settings/spinner_thread", spinner_thread, 1);
    // ROS_INFO("spinner_thread is %d.", spinner_thread);
    ros::AsyncSpinner spinner(spinner_thread); // Use n threads
    spinner.start();
    ros::waitForShutdown();
    ROS_INFO("Hello world, the vrmapping is closing.");
    return 0;
}