#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <cstdint>
#include <ignition/gazebo/System.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sdf/Imu.hh>
#include <sdf/sdf.hh>

int cnt = 0;
double last_fps_time;
image_transport::Publisher pubThermalImage;
ros::Publisher IMU_read_pub ;

double linearResolution = 0.01;

void OnImage(const ignition::msgs::Image &_msg) {
    // convert the serialized image data to 16-bit temperature values
    unsigned int thermalSamples = _msg.width() * _msg.height();
    unsigned int thermalWidth = _msg.width();
    unsigned int thermalHeight = _msg.height();
    unsigned int thermalBufferSize = thermalSamples * sizeof(uint16_t);
    float maxtemp = 0;
    auto *thermalBuffer = new uint16_t[thermalSamples];
    memcpy(thermalBuffer, _msg.data().c_str(), thermalBufferSize);
    for (auto r = 0; r < _msg.height(); ++r) {
        // need to figure out the row offset in order to mimic 2D array access
        // with 1D array indexing
        auto rowOffset = r * _msg.width();
        for (auto c = 0; c < _msg.width(); ++c) {
            // convert the 16-bit value to Kelvin via the camera's linearResolution
            auto temp = thermalBuffer[rowOffset + c] * linearResolution;
            // do something useful with the temperature (in Kelvin) here
            if (maxtemp < temp) {
                maxtemp = temp;
            }
        }
    }

    ROS_INFO("width:%d,height:%d", thermalWidth, thermalHeight);
    ROS_INFO("maxtemp:%f", maxtemp);
    cv::Mat Imageresult(_msg.height(), _msg.width(), CV_16UC1, thermalBuffer, 320 * 2);
    // cv::Mat Imageresult(_msg.width(), _msg.height(), CV_16UC1, thermalBuffer);
    // cv::Mat Imageresult(_msg.height(), _msg.width(), CV_8UC1, thermalBuffer);

    double fpstimestart = cv::getTickCount();
    ROS_INFO("fps:%lf", (cv::getTickFrequency() / (double)(fpstimestart - last_fps_time)));
    last_fps_time = fpstimestart;

    cnt++;
    // std::cout << " \"\/thermal_camera \" topic subscibed updated. " << cnt << std::endl;

    cv::imshow("result_win", Imageresult);
    cv::waitKey(1);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", Imageresult).toImageMsg();
    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", Imageresult).toImageMsg();
    msg->header.stamp = ros::Time::now();
    pubThermalImage.publish(msg);

    delete[] thermalBuffer;
}
void imuCb(const ignition::msgs::IMU &_msg) {
    std::string imuName;
    bool imuInitialized;
    bool imuMsgValid = true;
    std::mutex imuMsgMutex;
    // std::lock_guard<std::mutex> lock(this->imuMsgMutex);
    // const ::std::string name = _msg.entity_name();
    ignition::msgs::Quaternion Q_orientation = _msg.orientation();
    ignition::msgs::Vector3d angular_v = _msg.angular_velocity();
    ignition::msgs::Vector3d linear_acceleration = _msg.linear_acceleration();
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = "base_link";
    imu_data.orientation.x = Q_orientation.x();
    imu_data.orientation.y = Q_orientation.y();
    imu_data.orientation.z = Q_orientation.z();
    imu_data.orientation.w = Q_orientation.w();
    imu_data.angular_velocity.x = angular_v.x();
    imu_data.angular_velocity.y = angular_v.y();
    imu_data.angular_velocity.z = angular_v.z();
    imu_data.linear_acceleration.x = linear_acceleration.x();
    imu_data.linear_acceleration.y = linear_acceleration.y();
    imu_data.linear_acceleration.z = linear_acceleration.z();
    IMU_read_pub.publish(imu_data);
}
int main(int argc, char **argv) {
    ignition::transport::Node thermal_node;
    ignition::transport::Node imu_node;
    if (!thermal_node.Subscribe("/thermal_camera", &OnImage)) {
        ROS_ERROR("Error subscribing to the thermal camera topic");
        std::cerr << "Error subscribing to the thermal camera topic" << std::endl;
        return -1;
    }
    if (!imu_node.Subscribe("/imu_topic", &imuCb)) {
        ROS_ERROR("Error subscribing to the IMU topic");
        std::cerr << "Error subscribing to the IMU topic" << std::endl;
        return -1;
    }
    std::cout << " thermal_imu_vio_example start! " << std::endl;
    ros::init(argc, argv, "thermal_camera_example_pkg");
    ros::NodeHandle rnh;
    IMU_read_pub = rnh.advertise<sensor_msgs::Imu>("imu/data", 100);
    image_transport::ImageTransport it(rnh);
    pubThermalImage = it.advertise("/thermal_camera_example", 100);

    ignition::transport::waitForShutdown();
    return 0;
}
