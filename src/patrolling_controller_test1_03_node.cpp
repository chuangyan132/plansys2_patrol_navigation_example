// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

struct WaypointGroup {
    std::string action;
    std::vector<std::string> waypoints;
};

class PatrollingController : public rclcpp::Node
{
public:
  PatrollingController()
  : rclcpp::Node("patrolling_controller_test1_03"), 
    state_(STARTING),
    current_group_(0), 
    current_waypoint_(0)
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

        try {
            YAML::Node config = YAML::LoadFile(config_path);
            for (const auto& group : config["waypoint_sequence"]) {
                WaypointGroup wp_group;
                wp_group.action = group["group"].as<std::string>();
                wp_group.waypoints = group["waypoints"].as<std::vector<std::string>>();
                waypoint_groups_.push_back(wp_group);
            }
            RCLCPP_INFO(get_logger(), "Loaded %zu waypoint groups from config file: %s", waypoint_groups_.size(), config_file.c_str());
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Error reading config file %s: %s", config_file.c_str(), e.what());
        }
    }
  void step()
  {
    switch (state_)
    {
      case STARTING:
      {
        if (current_group_ >= waypoint_groups_.size()) {
          RCLCPP_INFO(get_logger(), "All waypoint groups completed");
          state_ = STOP;
          return;
        }
        const auto& current_group = waypoint_groups_[current_group_];
        const auto& current_waypoint = current_group.waypoints[current_waypoint_];
        setGoalForWaypoint(current_waypoint, current_group.action);
        
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto start_time = std::chrono::high_resolution_clock::now();
        auto plan = planner_client_->getPlan(domain, problem);
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> planning_time = end_time - start_time;
        total_planning_time_ += planning_time.count();
        if (plan.has_value())
        {
          if (executor_client_->start_plan_execution(plan.value()))
          {
            RCLCPP_INFO(get_logger(), "Plan execution started for waypoint: %s", current_waypoint.c_str());
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
            RCLCPP_INFO(get_logger(), "Plan executed successfully");
            moveToNextWaypoint();
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
        displayTotalPlanningTime();
        cleanup();
        break;

      default:
        RCLCPP_ERROR(get_logger(), "Unexpected state");
        break;
    }
  }

private:
  typedef enum {STARTING, EXECUTING, STOP} StateType;
  StateType state_;

  std::vector<WaypointGroup> waypoint_groups_;
  size_t current_group_;
  size_t current_waypoint_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  double total_planning_time_;
  void cleanup()
  {
    RCLCPP_INFO(get_logger(), "Cleaning up and shutting down");
    // Ensure all clients are reset to release resources
    domain_expert_.reset();
    planner_client_.reset();
    problem_expert_.reset();
    executor_client_.reset();
  }

/*
 bool patrolWaypoint(const std::string& current_wp, const std::string& next_wp, const std::string& action)
  {
    RCLCPP_INFO(get_logger(), "Planning for waypoint: %s", next_wp.c_str());

    // Cleaning up
    problem_expert_->removePredicate(plansys2::Predicate("(patrolled " + current_wp + ")"));
    problem_expert_->removePredicate(plansys2::Predicate("(is_shot " + current_wp + ")"));

    // Set the goal for next state
    std::string goal;
    if (action == "shot_only") {
      goal = "(and(is_shot " + next_wp + "))";
    } else if (action == "patrol_only") {
      goal = "(and(patrolled " + next_wp + "))";
    } else if (action == "patrol_and_shot") {
      goal = "(and(patrolled " + next_wp + ")(is_shot " + next_wp + "))";
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown action type: %s", action.c_str());
      return false;
    }
    
    problem_expert_->setGoal(plansys2::Goal(goal));

    auto plan = computePlan();
    if (plan.has_value()) {
      executor_client_->start_plan_execution(plan.value());
      RCLCPP_INFO(get_logger(), "Plan found and execution started for %s", next_wp.c_str());
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "Could not find plan for %s", next_wp.c_str());
      return false;
    }
  }

  std::optional<plansys2_msgs::msg::Plan> computePlan()
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_time = end_time - start_time;
    total_planning_time_ += planning_time;
    std::cout << "Planning time: " << planning_time.count() << " seconds" << std::endl;

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
    }

    return plan;
  }
*/
  void setGoalForWaypoint(const std::string& waypoint, const std::string& action)
    {
      std::string goal;
      if (action == "shot_only") {
          goal = "(and(is_shot " + waypoint + "))";
      } else if (action == "patrol_only") {
          goal = "(and(patrolled " + waypoint + "))";
      } else if (action == "patrol_and_shot") {
          goal = "(and(patrolled " + waypoint + ")(is_shot " + waypoint + "))";
      }
      problem_expert_->setGoal(plansys2::Goal(goal));
    }

  void moveToNextWaypoint()
  {
    current_waypoint_++;
    if (current_waypoint_ >= waypoint_groups_[current_group_].waypoints.size()) {
      current_group_++;
      current_waypoint_ = 0;
    }
    state_ = STARTING;
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
