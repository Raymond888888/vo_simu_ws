#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

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

double linearResolution = 0.01;

void OnImage(const ignition::msgs::Image &_msg) {
    // convert the serialized image data to 16-bit temperature values
    unsigned int thermalSamples = _msg.width() * _msg.height();
    unsigned int thermalWidth = _msg.width();
    unsigned int thermalHeight = _msg.height();
    unsigned int thermalBufferSize = thermalSamples * sizeof(uint16_t);
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
            // 在这里对温度（开尔文）做一些有用的事情
        }
    }

    cv::Mat Imageresult(_msg.height(), _msg.width(), CV_16UC1, thermalBuffer);
    // std::cout << "height" << _msg.height() << "width" << _msg.width() << std::endl;

    double fpstimestart = cv::getTickCount();
    std::cout << "fps:" << (cv::getTickFrequency() / (double)(fpstimestart - last_fps_time)) << std::endl;
    last_fps_time = fpstimestart;

    cnt++;
    // std::cout << " \"\/thermal_camera \" topic subscibed updated. " << cnt << std::endl;

    cv::imshow("result_win", Imageresult);
    cv::waitKey(1);

    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", Imageresult).toImageMsg();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", Imageresult).toImageMsg();
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
    // std::cout << "name=" << name << std::endl;
    ignition::msgs::Vector3d angular_v = _msg.angular_velocity();
    std::cout << "angular_velocity:" << angular_v.x() << std::endl;
    ignition::msgs::Quaternion Q_orientation = _msg.orientation();
    std::cout << "Q_orientation x:" << Q_orientation.x() << std::endl;
    std::cout << "Q_orientation y:" << Q_orientation.y() << std::endl;
    std::cout << "Q_orientation z:" << Q_orientation.z() << std::endl;
    std::cout << "Q_orientation w:" << Q_orientation.w() << std::endl;
}
int main(int argc, char **argv) {
    ignition::transport::Node thermal_node;
    ignition::transport::Node imu_node;
    if (!thermal_node.Subscribe("/thermal_camera", &OnImage)) {
        std::cerr << "Error subscribing to the thermal camera topic" << std::endl;
        return -1;
    }
    if (!imu_node.Subscribe("/imu_topic", &imuCb)) {
        std::cerr << "Error subscribing to the IMU topic" << std::endl;
        return -1;
    }
    std::cout << " thermal_imu_vio_example start! " << std::endl;
    ros::init(argc, argv, "thermal_camera_example_pkg");
    ros::NodeHandle rnh;
    image_transport::ImageTransport it(rnh);
    pubThermalImage = it.advertise("/thermal_camera_example", 1);

    ignition::transport::waitForShutdown();
    return 0;
}
