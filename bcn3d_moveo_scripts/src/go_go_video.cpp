#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RealSensePublisher : public rclcpp::Node {
public:
    RealSensePublisher() : Node("realsense_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", 10);
        initCamera();
    }

private:
    void initCamera() {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        pipe.start(cfg);

        while (rclcpp::ok()) {
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::frame color_frame = frames.get_color_frame();

            // Проверка на получение нового кадра
            if (!color_frame) {
                continue;
            }

            // Конвертация в OpenCV матрицу
            cv::Mat image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
            publisher_->publish(*message);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rs2::pipeline pipe;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealSensePublisher>());
    rclcpp::shutdown();
    return 0;
}
