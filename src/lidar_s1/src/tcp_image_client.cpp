#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <stdexcept>
#include <fcntl.h>

using ImageMsg = sensor_msgs::msg::Image;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
using NodeSharedPtr = std::shared_ptr<rclcpp::Node>;

class TCPImageClient {
public:
    explicit TCPImageClient(NodeSharedPtr node,
                           const std::string& server_ip,
                           int server_port,
                           const std::string& topic_prefix,
                           bool use_system_time)
        : node_(node),
          it_(node),
          server_ip_(server_ip),
          server_port_(server_port),
          topic_prefix_(topic_prefix),
          use_system_time_(use_system_time),
          sockfd_(-1),
          running_(true) {
        initPublishers();
        startThreads();
    }

    ~TCPImageClient() {
        running_ = false;
        if (connect_thread_.joinable()) connect_thread_.join();
        if (receive_thread_.joinable()) receive_thread_.join();
        disconnect();
    }

private:
    void initPublishers() {
        const std::string topic_root = "/" + topic_prefix_ + "/";
        top_half_pub_ = it_.advertise(topic_root + "left/image_raw", 1);
        bottom_half_pub_ = it_.advertise(topic_root + "right/image_raw", 1);
        logInfo("Raw image topics: %sleft/image_raw, %sright/image_raw",
                topic_root.c_str(), topic_root.c_str());
    }

    void startThreads() {
        connect_thread_ = std::thread(&TCPImageClient::reconnectLoop, this);
        receive_thread_ = std::thread(&TCPImageClient::receiveImages, this);
    }

    bool connectWithTimeout() {
        struct sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(server_port_);
        inet_pton(AF_INET, server_ip_.c_str(), &serv_addr.sin_addr);

        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) return false;

        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        int conn_result = connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
        if (conn_result < 0 && errno != EINPROGRESS) {
            close(sock);
            return false;
        }

        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(sock, &write_fds);

        struct timeval tv {.tv_sec = 3, .tv_usec = 0};

        int ready = select(sock + 1, nullptr, &write_fds, nullptr, &tv);
        if (ready <= 0) {
            close(sock);
            return false;
        }

        int error = 0;
        socklen_t len = sizeof(error);
        getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);
        if (error != 0) {
            close(sock);
            return false;
        }

        fcntl(sock, F_SETFL, flags);

        {
            std::lock_guard<std::mutex> lock(sock_mutex_);
            sockfd_ = sock;
        }

        logInfo("Connected to %s:%d", server_ip_.c_str(), server_port_);
        return true;
    }

    void disconnect() {
        std::lock_guard<std::mutex> lock(sock_mutex_);
        if (sockfd_ != -1) {
            close(sockfd_);
            sockfd_ = -1;
            logWarn("Disconnected from server");
        }
    }

    bool isConnected() {
        std::lock_guard<std::mutex> lock(sock_mutex_);
        return sockfd_ != -1;
    }

    void reconnectLoop() {
        while (running_) {
            if (!isConnected()) {
                if (connectWithTimeout()) {
                    // 成功连接
                } else if (running_) {
                    logWarn("Connection failed, retrying in 1 second...");
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    bool readWithTimeout(void* buffer, size_t length, int timeout_sec) {
        fd_set read_fds;
        FD_ZERO(&read_fds);

        int sock;
        {
            std::lock_guard<std::mutex> lock(sock_mutex_);
            if (sockfd_ == -1) return false;
            sock = sockfd_;
            FD_SET(sock, &read_fds);
        }

        struct timeval tv {.tv_sec = timeout_sec, .tv_usec = 0};

        size_t bytes_received = 0;
        while (bytes_received < length && running_) {
            int ready = select(sock + 1, &read_fds, nullptr, nullptr, &tv);
            if (ready <= 0) return false;

            ssize_t n = recv(sock, static_cast<uint8_t*>(buffer) + bytes_received,
                             length - bytes_received, 0);
            if (n <= 0) return false;

            bytes_received += n;
        }
        return true;
    }

    void receiveImages() {
        while (running_) {
            if (!isConnected()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            try {
                uint32_t data_size;
                if (!readWithTimeout(&data_size, sizeof(data_size), 1)) {
                    disconnect();
                    continue;
                }

                data_size = ntohl(data_size);

                std::vector<uint8_t> buffer(data_size);
                if (!readWithTimeout(buffer.data(), data_size, 3)) {
                    logWarn("Failed to receive complete image data");
                    disconnect();
                    continue;
                }

                CompressedImageMsg compressed_img;
                deserializeROSMessage(compressed_img, buffer);

                cv::Mat cv_img = cv::imdecode(compressed_img.data, cv::IMREAD_COLOR);
                if (cv_img.empty()) {
                    logWarn("Failed to decode image");
                    continue;
                }

                int height = cv_img.rows;
                int width = cv_img.cols;
                int split_point = height / 2;

                cv::Mat top_half = cv_img(cv::Rect(0, 0, width, split_point));
                cv::Mat bottom_half = cv_img(cv::Rect(0, split_point, width, height - split_point));

                // Capture a single timestamp for both halves so they share
                // the same stamp regardless of publish() latency.
                rclcpp::Time capture_stamp =
                    use_system_time_ ? node_->now()
                                     : rclcpp::Time(compressed_img.header.stamp);

                publishImage(top_half,    compressed_img.header, "top_half",    capture_stamp);
                publishImage(bottom_half, compressed_img.header, "bottom_half", capture_stamp);
            } catch (const std::exception& e) {
                logError("Error processing image: %s", e.what());
                disconnect();
            }
        }
    }

    void deserializeROSMessage(CompressedImageMsg& msg,
                               const std::vector<uint8_t>& buffer) {
        const uint8_t* data = buffer.data();
        size_t offset = 0;

        uint32_t seq;
        memcpy(&seq, data + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        int32_t sec, nsec;
        memcpy(&sec, data + offset, sizeof(int32_t));
        offset += sizeof(int32_t);
        memcpy(&nsec, data + offset, sizeof(int32_t));
        offset += sizeof(int32_t);

        uint32_t frame_id_len;
        memcpy(&frame_id_len, data + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        msg.header.frame_id.assign(reinterpret_cast<const char*>(data + offset), frame_id_len);
        offset += frame_id_len;

        uint32_t format_len;
        memcpy(&format_len, data + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        msg.format.assign(reinterpret_cast<const char*>(data + offset), format_len);
        offset += format_len;

        uint32_t data_len;
        memcpy(&data_len, data + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        msg.data.assign(data + offset, data + offset + data_len);

        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nsec;
    }

    void logInfo(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        RCLCPP_INFO(node_->get_logger(), "%s", buffer);
    }

    void logWarn(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        RCLCPP_WARN(node_->get_logger(), "%s", buffer);
    }

    void logError(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        RCLCPP_ERROR(node_->get_logger(), "%s", buffer);
    }

    void publishImage(const cv::Mat& image,
                     const std_msgs::msg::Header& header,
                     const std::string& name,
                     const rclcpp::Time& stamp) {
        try {
            cv_bridge::CvImage cv_image;
            cv_image.header.stamp = stamp;
            cv_image.header.frame_id = header.frame_id;
            cv_image.encoding = sensor_msgs::image_encodings::BGR8;
            cv_image.image = image;

            auto img_msg = cv_image.toImageMsg();

            if (name == "top_half") {
                top_half_pub_.publish(img_msg);
            } else {
                bottom_half_pub_.publish(img_msg);
            }
        } catch (const std::exception& e) {
            logError("Error publishing %s image: %s", name.c_str(), e.what());
        }
    }

    std::string server_ip_;
    int server_port_;
    std::string topic_prefix_;
    bool use_system_time_;
    std::atomic<int> sockfd_;
    std::atomic<bool> running_;

    NodeSharedPtr node_;
    image_transport::ImageTransport it_;
    image_transport::Publisher top_half_pub_;
    image_transport::Publisher bottom_half_pub_;

    std::mutex sock_mutex_;
    std::thread connect_thread_;
    std::thread receive_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tcp_image_client");

    std::string server_ip = node->declare_parameter("server_ip", "192.168.2.2");
    int server_port = node->declare_parameter("server_port", 8888);
    std::string topic_prefix = node->declare_parameter<std::string>("topic_prefix", "s1_01");
    bool use_system_time = node->declare_parameter<bool>("use_system_time", false);

    RCLCPP_INFO(node->get_logger(), "Starting TCP image client. Server: %s:%d, topic_prefix: /%s/, use_system_time: %s",
                server_ip.c_str(), server_port, topic_prefix.c_str(), use_system_time ? "true" : "false");

    try {
        TCPImageClient client(node, server_ip, server_port, topic_prefix, use_system_time);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Initialization failed: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
