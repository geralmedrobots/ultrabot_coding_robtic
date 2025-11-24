#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>

namespace somanet
{
class ObjectSegmentationNode : public rclcpp::Node
{
public:
  ObjectSegmentationNode()
  : rclcpp::Node("object_segmentation_node")
  {
    depth_topic_ = this->declare_parameter<std::string>(
      "depth_topic", "/realsense/realsense2_camera/depth/image_rect_raw");
    mask_topic_ = this->declare_parameter<std::string>(
      "mask_topic", "/perception/segmentation_mask");
    detections_topic_ = this->declare_parameter<std::string>(
      "detections_topic", "/perception/detections");
    camera_info_topic_ = this->declare_parameter<std::string>(
      "camera_info_topic", "/realsense/realsense2_camera/depth/camera_info");
    use_camera_info_ = this->declare_parameter<bool>("use_camera_info", true);
    fx_override_ = this->declare_parameter<double>("fx", 0.0);
    fy_override_ = this->declare_parameter<double>("fy", 0.0);
    cx_override_ = this->declare_parameter<double>("cx", 0.0);
    cy_override_ = this->declare_parameter<double>("cy", 0.0);
    min_depth_m_ = this->declare_parameter<double>("min_depth_m", 0.25);
    max_depth_m_ = this->declare_parameter<double>("max_depth_m", 5.0);
    min_area_px_ = this->declare_parameter<int>("min_area_px", 80);
    median_kernel_size_ = this->declare_parameter<int>("median_kernel_size", 5);
    morph_kernel_size_ = this->declare_parameter<int>("morph_kernel_size", 3);
    publish_debug_mask_ = this->declare_parameter<bool>("publish_debug_mask", true);

    validateAndClampParameters();

    auto sensor_qos = rclcpp::SensorDataQoS();
    image_transport::ImageTransport it(shared_from_this());
    depth_sub_ = it.subscribe(
      depth_topic_, 1, std::bind(&ObjectSegmentationNode::depthCallback, this, std::placeholders::_1),
      "raw", sensor_qos.get_rmw_qos_profile());

    if (use_camera_info_) {
      camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, 10,
        std::bind(&ObjectSegmentationNode::cameraInfoCallback, this, std::placeholders::_1));
    }

    if (publish_debug_mask_) {
      mask_pub_ = image_transport::create_publisher(this, mask_topic_);
    }
    detections_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
      detections_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "Object segmentation ready. depth_topic=%s mask_topic=%s detections_topic=%s",
      depth_topic_.c_str(), mask_topic_.c_str(), detections_topic_.c_str());
  }

private:
  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "cv_bridge conversion failed: %s", ex.what());
      return;
    }

    cv::Mat depth_meters;
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      cv_ptr->image.convertTo(depth_meters, CV_32FC1, 0.001);
    } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      depth_meters = cv_ptr->image;
    } else {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Unsupported depth encoding: %s", msg->encoding.c_str());
      return;
    }

    cv::Mat finite_mask = depth_meters == depth_meters;  // filter NaN
    cv::Mat positive_mask = depth_meters > 0.0f;
    cv::Mat interval_mask = (depth_meters >= min_depth_m_) & (depth_meters <= max_depth_m_);
    cv::Mat depth_mask = finite_mask & positive_mask & interval_mask;
    depth_mask.convertTo(depth_mask, CV_8UC1, 255.0);

    if (median_kernel_size_ > 1) {
      cv::medianBlur(depth_mask, depth_mask, median_kernel_size_);
    }

    if (morph_kernel_size_ > 1) {
      cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(morph_kernel_size_, morph_kernel_size_));
      cv::morphologyEx(depth_mask, depth_mask, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(depth_mask, depth_mask, cv::MORPH_CLOSE, kernel);
    }

    cv::Mat labels, stats, centroids;
    const int num_components = cv::connectedComponentsWithStats(
      depth_mask, labels, stats, centroids, 8, CV_16U);

    vision_msgs::msg::Detection2DArray detections;
    detections.header = msg->header;

    for (int i = 1; i < num_components; ++i) {
      const int area = stats.at<int>(i, cv::CC_STAT_AREA);
      if (area < min_area_px_) {
        continue;
      }

      const int x = stats.at<int>(i, cv::CC_STAT_LEFT);
      const int y = stats.at<int>(i, cv::CC_STAT_TOP);
      const int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
      const int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

      DepthStats stats_out;
      if (!computeDepthStatistics(depth_meters, labels, i, stats_out)) {
        continue;
      }

      vision_msgs::msg::Detection2D detection;
      detection.header = msg->header;
      detection.bbox.center.position.x = static_cast<double>(x) + width * 0.5;
      detection.bbox.center.position.y = static_cast<double>(y) + height * 0.5;
      detection.bbox.center.theta = 0.0;
      detection.bbox.size_x = static_cast<double>(width);
      detection.bbox.size_y = static_cast<double>(height);

      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = "obstacle";
      const double area_score = static_cast<double>(area) / (msg->width * msg->height);
      const double proximity_score = 1.0 - std::min(1.0, stats_out.median_depth / max_depth_m_);
      hypothesis.hypothesis.score = std::clamp(0.25 * area_score + 0.75 * proximity_score, 0.0, 1.0);

      geometry_msgs::msg::Pose & pose = hypothesis.pose.pose;
      if (tryProjectTo3D(detection, stats_out.median_depth, pose)) {
        // covariance: simple depth variance propagated to XYZ, orientation identity
        hypothesis.pose.covariance = {
          stats_out.variance, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, stats_out.variance, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, stats_out.variance, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1e-3};
      } else {
        pose.position.z = stats_out.median_depth;
        pose.orientation.w = 1.0;
      }
      hypothesis.pose.pose.orientation.w = 1.0;
      detection.results.emplace_back(std::move(hypothesis));

      detections.detections.emplace_back(std::move(detection));
    }

    if (!detections.detections.empty()) {
      detections_pub_->publish(detections);
    }

    if (publish_debug_mask_ && mask_pub_.getNumSubscribers() > 0) {
      auto mask_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, depth_mask)
                        .toImageMsg();
      mask_pub_.publish(mask_msg);
    }
  }

  struct DepthStats
  {
    double median_depth{0.0};
    double mean_depth{0.0};
    double variance{0.0};
  };

  bool computeDepthStatistics(
    const cv::Mat & depth_meters, const cv::Mat & labels, int component_idx, DepthStats & stats_out)
  {
    std::vector<float> depth_values;
    depth_values.reserve(static_cast<size_t>(labels.rows * labels.cols / 8));

    for (int r = 0; r < labels.rows; ++r) {
      const uint16_t * label_ptr = labels.ptr<uint16_t>(r);
      const float * depth_ptr = depth_meters.ptr<float>(r);
      for (int c = 0; c < labels.cols; ++c) {
        if (label_ptr[c] == component_idx) {
          const float depth = depth_ptr[c];
          if (std::isfinite(depth) && depth > 0.0f) {
            depth_values.push_back(depth);
          }
        }
      }
    }

    if (depth_values.empty()) {
      return false;
    }

    const size_t mid = depth_values.size() / 2;
    std::nth_element(depth_values.begin(), depth_values.begin() + mid, depth_values.end());
    stats_out.median_depth = depth_values[mid];
    const double sum = std::accumulate(depth_values.begin(), depth_values.end(), 0.0);
    stats_out.mean_depth = sum / depth_values.size();
    double accum = 0.0;
    for (const float v : depth_values) {
      const double diff = v - stats_out.mean_depth;
      accum += diff * diff;
    }
    stats_out.variance = accum / static_cast<double>(depth_values.size());
    return true;
  }

  void validateAndClampParameters()
  {
    if (min_depth_m_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "min_depth_m must be > 0. Resetting to 0.1 m");
      min_depth_m_ = 0.1;
    }
    if (max_depth_m_ <= min_depth_m_) {
      RCLCPP_WARN(
        this->get_logger(), "max_depth_m must be > min_depth_m. Adjusting to %.2f m",
        min_depth_m_ + 0.25);
      max_depth_m_ = min_depth_m_ + 0.25;
    }
    if (min_area_px_ < 1) {
      RCLCPP_WARN(this->get_logger(), "min_area_px must be >= 1. Resetting to 50 px");
      min_area_px_ = 50;
    }

    auto normalize_kernel = [](int value, int fallback) {
      if (value < 1) {
        value = fallback;
      }
      if (value % 2 == 0) {
        ++value;
      }
      return value;
    };
    median_kernel_size_ = normalize_kernel(median_kernel_size_, 5);
    morph_kernel_size_ = normalize_kernel(morph_kernel_size_, 3);
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
  {
    camera_info_ = msg;
  }

  bool tryProjectTo3D(
    const vision_msgs::msg::Detection2D & detection, double depth_m,
    geometry_msgs::msg::Pose & pose_out)
  {
    const auto camera_info = camera_info_;
    double fx = fx_override_;
    double fy = fy_override_;
    double cx = cx_override_;
    double cy = cy_override_;

    if (use_camera_info_) {
      if (!camera_info) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Waiting for camera_info to project detections to 3D");
        return false;
      }
      fx = camera_info->k[0];
      fy = camera_info->k[4];
      cx = camera_info->k[2];
      cy = camera_info->k[5];
    }

    if (fx <= 0.0 || fy <= 0.0) {
      return false;
    }

    const double u = detection.bbox.center.position.x;
    const double v = detection.bbox.center.position.y;

    pose_out.position.z = depth_m;
    pose_out.position.x = (u - cx) * depth_m / fx;
    pose_out.position.y = (v - cy) * depth_m / fy;
    pose_out.orientation.w = 1.0;
    return true;
  }

  std::string depth_topic_;
  std::string mask_topic_;
  std::string detections_topic_;
  bool use_camera_info_;
  std::string camera_info_topic_;
  double fx_override_;
  double fy_override_;
  double cx_override_;
  double cy_override_;
  double min_depth_m_;
  double max_depth_m_;
  int min_area_px_;
  int median_kernel_size_;
  int morph_kernel_size_;
  bool publish_debug_mask_;

  image_transport::Subscriber depth_sub_;
  image_transport::Publisher mask_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
};
}  // namespace somanet

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<somanet::ObjectSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}

