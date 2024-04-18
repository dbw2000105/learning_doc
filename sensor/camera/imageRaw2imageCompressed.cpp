#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ros::Publisher compressed_image_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    // 将图像消息转换为 OpenCV 图像格式
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 创建一个压缩图像消息对象
    sensor_msgs::CompressedImage compressed_msg;

    // 将 OpenCV 图像压缩为 JPEG 格式
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80); // 设置 JPEG 压缩质量
    cv::imencode(".jpg", cv_ptr->image, compressed_msg.data, compression_params);

    // 设置压缩图像消息的 header
    compressed_msg.header = image_msg->header;
    compressed_msg.format = "jpeg"; // 设置图像格式为 JPEG

    // 发布压缩图像消息
    compressed_image_pub.publish(compressed_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_compressor_node");
    ros::NodeHandle nh;

    // 创建一个压缩图像消息发布器
    compressed_image_pub = nh.advertise<sensor_msgs::CompressedImage>("/stereo_left_node/left/compressed", 1);

    // 创建一个相机图像消息订阅器
    ros::Subscriber image_sub = nh.subscribe("/stereo_left_node/left", 1, imageCallback);

    ros::spin();

    return 0;
}

