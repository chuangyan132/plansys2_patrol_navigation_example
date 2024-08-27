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
#include <sstream>
#include <plansys2_pddl_parser/Utils.h>

#include <memory>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

class PatrollingController : public rclcpp::Node
{
public:
  PatrollingController()
  : rclcpp::Node("patrolling_controller"), state_(STARTING), current_goal_index_(0), total_planning_time_(0)
  {
  }

  void init(const std::string& config_file)
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge_case1(config_file);
    // init_knowledge();
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

  void init_knowledge_case1(const std::string& config_file)
  {
    try 
    {
      load_config(config_file);
      load_instances(config_);
      load_predicates(config_);
      load_connections(config_);
      init_goals();
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file: %s", e.what());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading knowledge: %s", e.what());
    }
  }
  void step()
  {
    switch (state_) 
    {
      case STARTING:
      {
        if (current_goal_index_ >= goals_.size()) {
          RCLCPP_INFO(get_logger(), "All goals completed");
          displayTotalPlanningTime();
          state_ = STOP;
          return;
        }

        const auto& current_goal = goals_[current_goal_index_];

        RCLCPP_INFO(get_logger(), "Planning for goal: %s", current_goal.c_str());

        problem_expert_->setGoal(plansys2::Goal(current_goal));

        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();

        auto start_time = std::chrono::high_resolution_clock::now();
        auto plan = planner_client_->getPlan(domain, problem);
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> planning_time = end_time - start_time;
        total_planning_time_ += planning_time.count();

        RCLCPP_INFO(get_logger(), "Planning time for goal %zu: %.2f seconds", 
                    current_goal_index_, planning_time.count());

        if (plan.has_value())
        {
          if (executor_client_->start_plan_execution(plan.value()))
          {
            RCLCPP_INFO(get_logger(), "Plan execution started for goal %zu", 
                        current_goal_index_);
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
              RCLCPP_INFO(get_logger(), "Plan executed successfully for goal %zu", 
                          current_goal_index_);
              removePreviousGoalPredicates(goals_[current_goal_index_]);
              current_goal_index_++;
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
        {
          displayTotalPlanningTime();
        }
        break;
    }
  }

private:
  typedef enum {STARTING, EXECUTING, STOP
  } StateType;
  size_t current_goal_index_;
  StateType state_;
  std::string goal_string_;
  std::vector<std::string> goals_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  double total_planning_time_;

  std::string config_file_;
  YAML::Node config_;

  void load_config(const std::string& config_file)
  {
    std::string package_path = ament_index_cpp::get_package_share_directory("plansys2_patrol_navigation_example");
    config_file_ = package_path + "/tasks/" + config_file;

    RCLCPP_INFO(this->get_logger(), "Attempting to load config file: %s", config_file_.c_str());

    try 
    {
      config_ = YAML::LoadFile(config_file_);
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file: %s", e.what());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading config file: %s", e.what());
    }
  }
  void load_instances(const YAML::Node& config)
  {
    if (config["instances"]) {
      for (const auto& instance : config["instances"]) {
        std::string name = instance["name"].as<std::string>();
        std::string type = instance["type"].as<std::string>();
        problem_expert_->addInstance(plansys2::Instance{name, type});
        RCLCPP_INFO(this->get_logger(), "Added instance: %s of type %s", name.c_str(), type.c_str());
      }
    }
  }
  
  void load_predicates(const YAML::Node& config)
  {
    if (config["predicates"]) {
      for (const auto& predicate : config["predicates"]) {
        std::string pred_str = "(" + predicate["name"].as<std::string>();
        for (const auto& param : predicate["parameters"]) {
          pred_str += " " + param.as<std::string>();
        }
        pred_str += ")";
        problem_expert_->addPredicate(plansys2::Predicate(pred_str));
        RCLCPP_INFO(this->get_logger(), "Added predicate: %s", pred_str.c_str());
      }
  }
  }
  
  void load_connections(const YAML::Node& config)
  {
    if (config["connections"]) {
      for (const auto& connection : config["connections"]) {
        std::string wp1 = connection[0].as<std::string>();
        std::string wp2 = connection[1].as<std::string>();
        add_connection(wp1, wp2);
      }
    }
  }

  void add_connection(const std::string& wp1, const std::string& wp2)
{
  std::string full_wp1 = wp1 == "home" ? "wp_home" : 
                          wp1 == "charge" ? "wp_charge" : "wp_" + wp1;
  std::string full_wp2 = wp2 == "home" ? "wp_home" : 
                          wp2 == "charge" ? "wp_charge" : "wp_" + wp2;

  problem_expert_->addPredicate(plansys2::Predicate("(connected " + full_wp1 + " " + full_wp2 + ")"));
  problem_expert_->addPredicate(plansys2::Predicate("(connected " + full_wp2 + " " + full_wp1 + ")"));
  RCLCPP_INFO(this->get_logger(), "Added bidirectional connection between %s and %s", full_wp1.c_str(), full_wp2.c_str());
}

  void init_goals()
  {
    if (config_["goals"]) {
    for (const auto& goal_entry : config_["goals"]) {
      std::string goal_string = goal_entry.as<std::string>();
      std::istringstream iss(goal_string);
      std::string action, waypoint;
      std::string formatted_goal = "(and";
      
      while (iss >> action >> waypoint) {
        formatted_goal += " (" + action + " " + waypoint + ")";
      }
      
      formatted_goal += ")";
      goals_.push_back(formatted_goal);
      
      RCLCPP_INFO(this->get_logger(), "Added goal: %s", formatted_goal.c_str());
    }
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %zu goals from configuration", goals_.size());
  }

  void removePreviousGoalPredicates(const std::string& completed_goal)
  {
    // Parse the completed goal string to extract predicates
    std::istringstream iss(completed_goal);
    std::string token;
    while (std::getline(iss, token, '(')) {
      if (token.find(')') != std::string::npos) {
        // Remove the closing parenthesis
        token = token.substr(0, token.find(')'));
        
        // Split the predicate into action and waypoint
        std::istringstream predicate_stream(token);
        std::string action, waypoint;
        predicate_stream >> action >> waypoint;
        
        // Remove the predicate
        problem_expert_->removePredicate(plansys2::Predicate("(" + action + " " + waypoint + ")"));
        RCLCPP_INFO(get_logger(), "Removed predicate: (%s %s)", action.c_str(), waypoint.c_str());
      }
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