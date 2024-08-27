#include <memory>
#include <string>
#include <algorithm>
#include <filesystem>
#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;
class ShotAction : public plansys2::ActionExecutorClient
{
public:
  ShotAction() : plansys2::ActionExecutorClient("shot", 500ms)
  {
    progress_ = 0.0;
    image_received_ = false;
    image_shot_ = false;
    //hardcoded save_directory_
    save_directory_ = "/home/celltower1/warehouse_image";

    // Ensure the save directory exists
    if (!fs::exists(save_directory_)) {
      try {
        fs::create_directories(save_directory_);
      } catch (const fs::filesystem_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create save directory: %s", e.what());
      }
    }
    

    this->declare_parameter("image_topic", "/front_stereo_camera/left_rgb/image_raw");
    std::string image_topic = this->get_parameter("image_topic").as_string();

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, 10,
      std::bind(&ShotAction::image_callback, this, std::placeholders::_1));
  }
  
private:
  void do_work()
  {
    if(progress_ < 1.0)
    {
        if (progress_ == 0.0) {
        // First call to do_work, set up the action
        auto arguments = get_arguments();
        if (arguments.size() >= 2) {
            position_name_ = arguments[1]; 
            RCLCPP_INFO(this->get_logger(), "Preparing to save image for position: %s", position_name_.c_str());
            progress_ = 0.1;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error: argument position_name not set");
            finish(false, 0.0, "Error: argument position_name not set");
            return;
        }
        }

        if (!image_received_) 
        {
            // Waiting for an image
            progress_ = std::min(progress_ + 0.1, 0.9);
            send_feedback(progress_, "Waiting for image for " + position_name_);
        } else 
        {
            // Image received, save it
            try {
                fs::path filepath = fs::path(save_directory_) / (position_name_ + ".jpg");
                cv::imwrite(filepath.string(), latest_image_);
                
                RCLCPP_INFO(this->get_logger(), "Image saved to: %s", filepath.c_str());
                
                progress_ = 1.0;
                send_feedback(progress_, "Shot completed for " + position_name_);
                finish(true, 1.0, "Shot completed");
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
                finish(false, 0.0, "Failed to save image");
            }
            
            // Reset for the next shot
            image_received_ = false;
            progress_ = 0.0;
        
    }
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      latest_image_ = cv_ptr->image;
      image_received_ = true;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  float progress_;
  bool image_shot_;
  bool image_received_;
  std::string position_name_;
  std::string save_directory_;
  cv::Mat latest_image_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShotAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "shot"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}