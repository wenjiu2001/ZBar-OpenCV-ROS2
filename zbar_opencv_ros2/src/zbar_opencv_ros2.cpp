#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <zbar.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class BarcodeScanner : public rclcpp::Node {
public:
    BarcodeScanner() : Node("zbar_opencv_ros2") {
        std::string image_topic = this->declare_parameter<std::string>("image_topic", "image_raw");
        std::string barcode_topic = this->declare_parameter<std::string>("barcode_topic", "barcode");
    
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10, std::bind(&BarcodeScanner::image_callback, this, std::placeholders::_1));

        barcode_publisher_ = this->create_publisher<std_msgs::msg::String>(barcode_topic, 10);

        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

        zbar::Image zbar_image(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
        scanner_.scan(zbar_image);

        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
             symbol != zbar_image.symbol_end();
             ++symbol) {
            std_msgs::msg::String barcode_message;
            barcode_message.data = symbol->get_data();
            barcode_publisher_->publish(barcode_message);
            RCLCPP_INFO(this->get_logger(), barcode_message.data);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr barcode_publisher_;
    zbar::ImageScanner scanner_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BarcodeScanner>());
    rclcpp::shutdown();
    return 0;
}
