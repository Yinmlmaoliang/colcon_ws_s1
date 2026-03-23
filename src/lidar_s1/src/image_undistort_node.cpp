#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <fstream>
#include <memory>

struct CameraCalibration {
    int image_width;
    int image_height;
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    cv::Mat rectification_matrix;
    cv::Mat projection_matrix;
    std::string distortion_model;
};

class ImageUndistortNode : public rclcpp::Node {
public:
    ImageUndistortNode() : Node("image_undistort") {
        std::string config_package = this->declare_parameter<std::string>("config_package", "lidars1");
        std::string pkg_share = ament_index_cpp::get_package_share_directory(config_package);
        std::string config_dir = pkg_share + "/config/";
        topic_prefix_ = this->declare_parameter<std::string>("topic_prefix", "s1_01");
        std::string topic_root = "/" + topic_prefix_ + "/";

        loadCalibration(config_dir + "left_fisheye.yaml", left_fisheye_);
        loadCalibration(config_dir + "right_fisheye.yaml", right_fisheye_);
        loadCalibration(config_dir + "left_rectified.yaml", left_rectified_);
        loadCalibration(config_dir + "right_rectified.yaml", right_rectified_);

        buildUndistortMaps();
        buildCameraInfoMessages();

        left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_root + "left/camera_info_rect", 10);
        right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_root + "right/camera_info_rect", 10);
    }

    void initTransport() {
        image_transport::ImageTransport it(shared_from_this());
        std::string topic_root = "/" + topic_prefix_ + "/";
        left_rect_pub_ = it.advertise(topic_root + "left/image_rect", 1);
        right_rect_pub_ = it.advertise(topic_root + "right/image_rect", 1);

        left_sub_ = it.subscribe(topic_root + "left/image_raw", 1,
            &ImageUndistortNode::leftCallback, this);
        right_sub_ = it.subscribe(topic_root + "right/image_raw", 1,
            &ImageUndistortNode::rightCallback, this);

        RCLCPP_INFO(this->get_logger(), "Image undistort node started, topic prefix: /%s/", topic_prefix_.c_str());
    }

private:
    void buildUndistortMaps() {
        cv::Size img_size(left_fisheye_.image_width, left_fisheye_.image_height);

        cv::fisheye::initUndistortRectifyMap(
            left_fisheye_.camera_matrix,
            left_fisheye_.distortion_coefficients,
            left_fisheye_.rectification_matrix,
            left_rectified_.camera_matrix,
            img_size, CV_16SC2,
            left_map1_, left_map2_);

        cv::fisheye::initUndistortRectifyMap(
            right_fisheye_.camera_matrix,
            right_fisheye_.distortion_coefficients,
            right_fisheye_.rectification_matrix,
            right_rectified_.camera_matrix,
            img_size, CV_16SC2,
            right_map1_, right_map2_);

        RCLCPP_INFO(this->get_logger(), "Undistortion maps computed (%dx%d)",
                     img_size.width, img_size.height);
    }

    void buildCameraInfoMessages() {
        fillCameraInfo(left_info_msg_, left_rectified_);
        fillCameraInfo(right_info_msg_, right_rectified_);
    }

    void fillCameraInfo(sensor_msgs::msg::CameraInfo& info,
                        const CameraCalibration& cal) {
        info.width = cal.image_width;
        info.height = cal.image_height;
        info.distortion_model = cal.distortion_model;

        info.d.resize(cal.distortion_coefficients.total());
        for (size_t i = 0; i < cal.distortion_coefficients.total(); ++i)
            info.d[i] = cal.distortion_coefficients.at<double>(i);

        for (int i = 0; i < 9; ++i)
            info.k[i] = cal.camera_matrix.at<double>(i);

        for (int i = 0; i < 9; ++i)
            info.r[i] = cal.rectification_matrix.at<double>(i);

        for (int i = 0; i < 12; ++i)
            info.p[i] = cal.projection_matrix.at<double>(i);
    }

    void leftCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        processAndPublish(msg, left_map1_, left_map2_,
                          left_rect_pub_, left_info_pub_, left_info_msg_);
    }

    void rightCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        processAndPublish(msg, right_map1_, right_map2_,
                          right_rect_pub_, right_info_pub_, right_info_msg_);
    }

    void processAndPublish(
            const sensor_msgs::msg::Image::ConstSharedPtr& msg,
            const cv::Mat& map1, const cv::Mat& map2,
            image_transport::Publisher& img_pub,
            rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& info_pub,
            sensor_msgs::msg::CameraInfo& info_template) {
        try {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

            cv::Mat rectified;
            cv::remap(cv_ptr->image, rectified, map1, map2, cv::INTER_LINEAR);

            cv_bridge::CvImage out;
            out.header = msg->header;
            out.encoding = sensor_msgs::image_encodings::BGR8;
            out.image = rectified;
            img_pub.publish(out.toImageMsg());

            info_template.header = msg->header;
            info_pub->publish(info_template);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Undistort error: %s", e.what());
        }
    }

    // --- YAML calibration loader ---
    static bool parseMatrixField(const std::string& content,
                                 const std::string& field_name,
                                 int expected_rows, int expected_cols,
                                 cv::Mat& out) {
        std::string key = field_name + ":";
        size_t pos = content.find(key);
        if (pos == std::string::npos) return false;

        size_t data_pos = content.find("data:", pos);
        if (data_pos == std::string::npos) return false;

        size_t bracket_start = content.find('[', data_pos);
        size_t bracket_end = content.find(']', bracket_start);
        if (bracket_start == std::string::npos || bracket_end == std::string::npos)
            return false;

        std::string nums = content.substr(bracket_start + 1,
                                          bracket_end - bracket_start - 1);

        std::vector<double> values;
        std::stringstream ss(nums);
        std::string token;
        while (std::getline(ss, token, ',')) {
            try { values.push_back(std::stod(token)); }
            catch (...) { continue; }
        }

        if (static_cast<int>(values.size()) != expected_rows * expected_cols)
            return false;

        out = cv::Mat(expected_rows, expected_cols, CV_64F);
        memcpy(out.data, values.data(), values.size() * sizeof(double));
        return true;
    }

    static int parseIntField(const std::string& content,
                             const std::string& field_name) {
        std::string key = field_name + ":";
        size_t pos = content.find(key);
        if (pos == std::string::npos) return 0;
        size_t val_start = pos + key.size();
        return std::stoi(content.substr(val_start, content.find('\n', val_start) - val_start));
    }

    static std::string parseStringField(const std::string& content,
                                        const std::string& field_name) {
        std::string key = field_name + ":";
        size_t pos = content.find(key);
        if (pos == std::string::npos) return "";
        size_t val_start = pos + key.size();
        size_t val_end = content.find('\n', val_start);
        std::string val = content.substr(val_start, val_end - val_start);
        size_t first = val.find_first_not_of(" \t");
        size_t last = val.find_last_not_of(" \t\r");
        return (first == std::string::npos) ? "" : val.substr(first, last - first + 1);
    }

    void loadCalibration(const std::string& path, CameraCalibration& cal) {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_FATAL(this->get_logger(), "Cannot open calibration: %s", path.c_str());
            throw std::runtime_error("Cannot open " + path);
        }

        std::string content((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());

        cal.image_width = parseIntField(content, "image_width");
        cal.image_height = parseIntField(content, "image_height");
        cal.distortion_model = parseStringField(content, "distortion_model");

        int d_cols = (cal.distortion_model == "equidistant") ? 4 : 5;

        parseMatrixField(content, "camera_matrix", 3, 3, cal.camera_matrix);
        parseMatrixField(content, "distortion_coefficients", 1, d_cols,
                         cal.distortion_coefficients);
        parseMatrixField(content, "rectification_matrix", 3, 3,
                         cal.rectification_matrix);
        parseMatrixField(content, "projection_matrix", 3, 4,
                         cal.projection_matrix);

        RCLCPP_INFO(this->get_logger(), "Loaded calibration: %s (%dx%d, %s)",
                    path.c_str(), cal.image_width, cal.image_height,
                    cal.distortion_model.c_str());
    }

    CameraCalibration left_fisheye_, right_fisheye_;
    CameraCalibration left_rectified_, right_rectified_;

    cv::Mat left_map1_, left_map2_;
    cv::Mat right_map1_, right_map2_;
    std::string topic_prefix_;

    sensor_msgs::msg::CameraInfo left_info_msg_, right_info_msg_;

    image_transport::Subscriber left_sub_, right_sub_;
    image_transport::Publisher left_rect_pub_, right_rect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageUndistortNode>();
    node->initTransport();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
