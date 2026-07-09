#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>
#include <filesystem>

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

//miei
#include "world_data_utils.hpp"
#include "debug.hpp"

using namespace std::chrono_literals;
using namespace std;

class PatrolAction : public plansys2::ActionExecutorClient
{
public:
  PatrolAction() : plansys2::ActionExecutorClient("patrol", 500ms){
    has_rgb_image_ = false;
    has_depth_image_ = false;
    rgb_saved_ = false;
    depth_saved_ = false;

    // Directory fissa di destinazione per gli snapshot
    snapshots_dir_ = "/root/limo_ws/src/limo_ros2/limo_planner/imgs";

    using namespace std::placeholders;

    subscriber_to_rgb_image_ = create_subscription<sensor_msgs::msg::Image>(
      "/limo_camera/image_raw", 10,
      std::bind(&PatrolAction::rgb_image_callback, this, _1));

    subscriber_to_depth_image_ = create_subscription<sensor_msgs::msg::Image>(
      "/limo_camera/depth_image", 10,
      std::bind(&PatrolAction::depth_image_callback, this, _1));
  }

  void rgb_image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    last_rgb_image_ = msg;
    has_rgb_image_ = true;
  }

  void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    last_depth_image_ = msg;
    has_depth_image_ = true;
  }

  void init_knowledge(){
    // The action is parameterized as (?r - robot ?wp - waypoint)
    wp_name_ = get_arguments()[1];

    RCLCPP_INFO(get_logger(), "Patrol: scatto foto al waypoint [%s]", wp_name_.c_str());

    has_rgb_image_ = false;
    has_depth_image_ = false;
    rgb_saved_ = false;
    depth_saved_ = false;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state){
    init_knowledge();
    send_feedback(0.0, "Patrol starting");
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  bool save_rgb_image(){
    if (!has_rgb_image_) {
      return false;
    }

    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_rgb_image_, "bgr8");
      std::string filepath = snapshots_dir_ + "/" + wp_name_ + "_rgb.png";
      cv::imwrite(filepath, cv_ptr->image);
      RCLCPP_INFO(get_logger(), "Salvata immagine RGB: %s", filepath.c_str());
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Errore salvataggio RGB: %s", e.what());
      return false;
    }
  }

  bool save_depth_image(){
    if (!has_depth_image_) {
      return false;
    }

    try {
      // Le depth image sono di solito 16UC1 (mm) o 32FC1 (m): le salviamo
      // cosi' come sono, senza convertirle, per non perdere precisione.
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_depth_image_, last_depth_image_->encoding);
      std::string filepath = snapshots_dir_ + "/" + wp_name_ + "_depth.png";
      cv::imwrite(filepath, cv_ptr->image);
      RCLCPP_INFO(get_logger(), "Salvata immagine depth: %s", filepath.c_str());
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Errore salvataggio depth: %s", e.what());
      return false;
    }
  }

  void do_work(){
    if (!rgb_saved_ && has_rgb_image_) {
      rgb_saved_ = save_rgb_image();
    }

    if (!depth_saved_ && has_depth_image_) {
      depth_saved_ = save_depth_image();
    }

    float progress = 0.0;
    if (rgb_saved_) progress += 0.5;
    if (depth_saved_) progress += 0.5;

    if (rgb_saved_ && depth_saved_) {
      finish(true, 1.0, "Patrol completed");
      return;
    }

    send_feedback(progress, "Aspettando le immagini");
  }

  //ROS subscribers / immagini
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> subscriber_to_rgb_image_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> subscriber_to_depth_image_;
  sensor_msgs::msg::Image::SharedPtr last_rgb_image_;
  sensor_msgs::msg::Image::SharedPtr last_depth_image_;
  bool has_rgb_image_;
  bool has_depth_image_;
  bool rgb_saved_;
  bool depth_saved_;

  //salvataggio
  std::string snapshots_dir_;
  std::string wp_name_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "patrol"));
  node->configure();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}