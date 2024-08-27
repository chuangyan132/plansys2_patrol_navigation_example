#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

struct WaypointActions {
    int waypoint;
    std::vector<std::string> actions;
};

class PatrollingController : public rclcpp::Node
{
public:
  PatrollingController()
  : rclcpp::Node("patrolling_controller_test1_4"), 
    state_(STARTING),
    current_waypoint_index_(0),
    current_action_index_(0),
    total_planning_time_(0)
  {
  }

  void init(const std::string& config_file)
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    
    init_knowledge();
    loadWaypointSequence(config_file);
  }

  void init_knowledge()
  {
    // Define Waypoints
    problem_expert_->addInstance(plansys2::Instance{"carter", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"wp_home", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_charge", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_4", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_5", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_6", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_7", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_8", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_9", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_10", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_11", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_12", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_13", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_14", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_15", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_16", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_17", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_18", "waypoint"});

    // Define Predicates
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at carter wp_home)"));
    problem_expert_->addPredicate(plansys2::Predicate("(charging_point_at wp_charge)"));
    problem_expert_->addPredicate(plansys2::Predicate("(battery_low carter)"));


    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_home wp_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_1 wp_home)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_home wp_charge)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_charge wp_home)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_1 wp_charge)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_charge wp_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_1 wp_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_2 wp_1)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_2 wp_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_3 wp_2)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_3 wp_4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_4 wp_3)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_4 wp_5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_5 wp_4)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_5 wp_6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_6 wp_5)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_6 wp_7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_7 wp_6)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_7 wp_8)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_8 wp_7)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_8 wp_9)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_9 wp_8)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_9 wp_10)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_10 wp_9)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_10 wp_11)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_11 wp_10)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_11 wp_12)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_12 wp_11)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_12 wp_13)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_13 wp_12)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_13 wp_14)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_14 wp_13)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_14 wp_15)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_15 wp_14)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_15 wp_16)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_16 wp_15)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_16 wp_home)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_home wp_16)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_16 wp_17)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_17 wp_16)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_17 wp_18)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_18 wp_17)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_18 wp_home)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_home wp_18)"));
  }

  void loadWaypointSequence(const std::string& config_file)
  {
    std::string package_path = ament_index_cpp::get_package_share_directory("plansys2_patrol_navigation_example");
    std::string config_path = package_path + "/tasks/" + config_file;

    RCLCPP_INFO(get_logger(), "Attempting to load config file: %s", config_path.c_str());

    try 
    {
      YAML::Node config = YAML::LoadFile(config_path);
      
      if (!config["waypoint_sequence"]) 
      {
        RCLCPP_ERROR(get_logger(), "Missing 'waypoint_sequence' key in YAML file");
        return;
      }

      for (const auto& wp : config["waypoint_sequence"]) {
        if (!wp["waypoint"] || !wp["actions"]) {
          RCLCPP_ERROR(get_logger(), "Waypoint entry is missing 'waypoint' or 'actions' key");
          continue;
        }

        int waypoint = wp["waypoint"].as<int>();
        std::vector<std::string> actions;
        
        if (wp["actions"].IsSequence()) {
          actions = wp["actions"].as<std::vector<std::string>>();
        } else if (wp["actions"].IsNull() || (wp["actions"].IsSequence() && wp["actions"].size() == 0)) {
          RCLCPP_INFO(get_logger(), "Waypoint %d has no actions (skippable)", waypoint);
        } else {
          RCLCPP_ERROR(get_logger(), "Invalid 'actions' format for waypoint %d", waypoint);
          continue;
        }

        waypoint_actions_.push_back({waypoint, actions});
        RCLCPP_INFO(get_logger(), "Loaded waypoint %d with %zu actions", waypoint, actions.size());
      }

      RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from config file: %s", waypoint_actions_.size(), config_file.c_str());
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(get_logger(), "YAML parsing error in file %s: %s", config_file.c_str(), e.what());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error reading config file %s: %s", config_file.c_str(), e.what());
    }
  }

  void step()
  {
    switch (state_)
    {
      case STARTING:
      {
        if (current_waypoint_index_ >= waypoint_actions_.size()) {
          RCLCPP_INFO(get_logger(), "All waypoints completed");
          displayTotalPlanningTime();
          state_ = STOP;
          return;
        }

        const auto& current_wp = waypoint_actions_[current_waypoint_index_];
        
        if (current_wp.actions.empty()) 
        {
          RCLCPP_INFO(get_logger(), "Skipping waypoint %d", current_wp.waypoint);
          moveToNextWaypoint();
          return;
        }

        RCLCPP_INFO(get_logger(), "Planning for waypoint %d with %zu actions", 
                            current_wp.waypoint, current_wp.actions.size());

        setGoalForWaypoint(current_wp.waypoint, current_wp.actions);
        
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        
        auto start_time = std::chrono::high_resolution_clock::now();
        auto plan = planner_client_->getPlan(domain, problem);
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> planning_time = end_time - start_time;
        total_planning_time_ += planning_time.count();
        
        RCLCPP_INFO(get_logger(), "Planning time for waypoint %d: %.2f seconds", 
                    current_wp.waypoint, planning_time.count());

        if (plan.has_value())
        {
          if (executor_client_->start_plan_execution(plan.value()))
          {
            RCLCPP_INFO(get_logger(), "Plan execution started for waypoint %d", 
                        current_wp.waypoint);
            state_ = EXECUTING;
          }
          else
          {
            RCLCPP_ERROR(get_logger(), "Failed to start plan execution");
            state_ = STOP;
          }
        }
        else
          {
            RCLCPP_ERROR(get_logger(), "Could not get plan");
            state_ = STOP;
          }
      }
        break;

      case EXECUTING:
      {
        auto feedback = executor_client_->getFeedBack();
        for (const auto & action_feedback : feedback.action_execution_status)
        {
          RCLCPP_INFO(get_logger(), "Action: %s, completion: %.2f%%",
                        action_feedback.action.c_str(),
                        action_feedback.completion * 100.0);
        }

        if (!executor_client_->execute_and_check_plan())
        {
          auto result = executor_client_->getResult();
          if (result.value().success)
          {
            RCLCPP_INFO(get_logger(), "Plan executed successfully for waypoint %d", 
                        waypoint_actions_[current_waypoint_index_].waypoint);
            moveToNextWaypoint();
            state_ = STARTING;
          }
          else
          {
            RCLCPP_ERROR(get_logger(), "Plan execution failed");
            state_ = STOP;
          }
        }
      }
        break;

      case STOP:
        // Do nothing
        break;

      default:
        RCLCPP_ERROR(get_logger(), "Unexpected state");
        break;
    }
  }
private:
  typedef enum {STARTING, EXECUTING, STOP} StateType;
  StateType state_;
  std::vector<WaypointActions> waypoint_actions_;
  size_t current_waypoint_index_;
  size_t current_action_index_;
  double total_planning_time_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  void setGoalForWaypoint(int waypoint, const std::vector<std::string>& actions)
  {
    std::string wp = "wp_" + std::to_string(waypoint);
    std::string goal = "(and";
        
    for (const auto& action : actions) {
      if (action == "shot") {
        goal += "(is_shot " + wp + ")";
      } else if (action == "patrol") {
        goal += "(patrolled " + wp + ")";
      } else {
        RCLCPP_WARN(get_logger(), "Unknown action type: %s for waypoint %d", action.c_str(), waypoint);
      }
    }
    
    goal += ")";
    
    RCLCPP_INFO(get_logger(), "Setting goal for waypoint %d: %s", waypoint, goal.c_str());
    problem_expert_->setGoal(plansys2::Goal(goal));
  }

  void moveToNextWaypoint()
  {
    current_waypoint_index_++;
    if (current_waypoint_index_ < waypoint_actions_.size()) {
      RCLCPP_INFO(get_logger(), "Moving to next waypoint: %d", waypoint_actions_[current_waypoint_index_].waypoint);
    } else {
      RCLCPP_INFO(get_logger(), "All waypoints completed");
    }
  }

  void displayTotalPlanningTime()
  {
    RCLCPP_INFO(get_logger(), "Total planning time for all waypoints: %.2f seconds", total_planning_time_);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();

  if (argc < 2) {
    RCLCPP_ERROR(node->get_logger(), "Usage: %s <yaml_file_name>", argv[0]);
    rclcpp::shutdown();
    return 1;
  }

  std::string config_file = argv[1];

  node->init(config_file);

  rclcpp::Rate rate(5);
  bool continue_execution = true;
  while (rclcpp::ok())
  {
    node->step();
    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}