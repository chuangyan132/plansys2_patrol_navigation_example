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

class PatrollingController : public rclcpp::Node
{
public:
  PatrollingController()
  : rclcpp::Node("patrolling_controller"), state_(STARTING)
  {
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
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

  void step()
  {
    switch (state_) {
      case STARTING:
        initializePatrol("wp_18");
        break;
      case PATROL_WP18:
        RCLCPP_INFO(get_logger(), "Patrolling waypoint 18 started");
        patrolWaypoint("wp_18", "wp_17", PATROL_WP17);
        break;
      case PATROL_WP17:
        patrolWaypoint("wp_17", "wp_16", PATROL_WP16);
        break;
      case PATROL_WP16:
        patrolWaypoint("wp_16", "wp_15", PATROL_WP15);
        break;
      case PATROL_WP15:
        patrolWaypoint("wp_15", "wp_14", PATROL_WP14);
        break;
      case PATROL_WP1:
        patrolWaypoint("wp_1", "wp_2", PATROL_WP2);
        break;
      case PATROL_WP3:
        patrolWaypoint("wp_3", "wp_1", STOP);
        break;
      case PATROL_WP12:
        patrolWaypoint("wp_12", "wp_6", PATROL_WP6);
        break;
      case PATROL_WP2:
        patrolWaypoint("wp_2", "wp_4", PATROL_WP4);
        break;
      case PATROL_WP5:
        patrolWaypoint("wp_5", "wp_1", PATROL_WP1);
        break;
      case PATROL_WP11:
        patrolWaypoint("wp_11", "wp_12", PATROL_WP12);
        break;
      case PATROL_WP4:
        patrolWaypoint("wp_4", "wp_3", PATROL_WP3);
        break;
      case PATROL_WP6:
        patrolWaypoint("wp_6", "wp_5", PATROL_WP5);
        break;
      case PATROL_WP10:
        patrolWaypoint("wp_10", "wp_8", PATROL_WP8);
        break;
      case PATROL_WP14:
        patrolWaypoint("wp_14", "wp_13", PATROL_WP13);
        break;
      case PATROL_WP7:
        patrolWaypoint("wp_7", "wp_11", PATROL_WP11);
        break;
      case PATROL_WP9:
        patrolWaypoint("wp_9", "wp_10", PATROL_WP10);
        break;
      case PATROL_WP13:
        patrolWaypoint("wp_13", "wp_9", PATROL_WP9);
        break;
      case PATROL_WP8:
        patrolWaypoint("wp_8", "wp_7", PATROL_WP7);
        break;
      case STOP:
        std::cout << "Total planning time for all waypoints: " << total_planning_time_.count() << " seconds" << std::endl;
        break;
    }
  }

private:
  typedef enum {STARTING, PATROL_WP1, PATROL_WP2, PATROL_WP3, PATROL_WP4, PATROL_WP5, 
  PATROL_WP6, PATROL_WP7, PATROL_WP8, PATROL_WP9, PATROL_WP10, PATROL_WP11, PATROL_WP12,
  PATROL_WP13, PATROL_WP14, PATROL_WP15, PATROL_WP16, PATROL_WP17, PATROL_WP18, STOP
  } StateType;
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  std::chrono::duration<double> total_planning_time_;

  void initializePatrol(const std::string& waypoint)
  {
    problem_expert_->setGoal(plansys2::Goal("(and(patrolled " + waypoint + "))"));
    auto plan = computePlan();
    if (plan.has_value()) {
      executor_client_->start_plan_execution(plan.value());
      state_ = PATROL_WP18;
    }
  }
  void patrolWaypoint(const std::string& current_wp, const std::string& next_wp, StateType next_state)
  {
    auto feedback = executor_client_->getFeedBack();

    for (const auto & action_feedback : feedback.action_execution_status) {
      std::cout << "[" << action_feedback.action << " " <<
        action_feedback.completion * 100.0 << "%]";
    }
    std::cout << std::endl;

    if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
      if (executor_client_->getResult().value().success) {
        std::cout << "Successfully finished " << std::endl;

        // Cleaning up
        problem_expert_->removePredicate(plansys2::Predicate("(patrolled " + current_wp + ")"));
        problem_expert_->removePredicate(plansys2::Predicate("(is_shot " + current_wp + ")"));

        // Set the goal for next state
        problem_expert_->setGoal(plansys2::Goal("(and(patrolled " + next_wp + "))"));

        auto plan = computePlan();
        if (plan.has_value()) {
          executor_client_->start_plan_execution(plan.value());
          state_ = next_state;  // Directly set the next state
        } else {
          state_ = STOP;
        }
      } else {
        for (const auto & action_feedback : feedback.action_execution_status) {
          if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
            std::cout << "[" << action_feedback.action << "] finished with error: " <<
              action_feedback.message_status << std::endl;
          }
        }

        // Replan
        auto plan = computePlan();
        if (plan.has_value()) {
          // executor_client_->start_plan_execution(plan.value());
        } else {
          state_ = STOP;
        }
      }
    }


  }

  void patrolWaypoint_execute(const std::string& current_wp, const std::string& next_wp, StateType next_state)
  {
    std::cout << "Planning for waypoint: " << next_wp << std::endl;

    // Cleaning up
    problem_expert_->removePredicate(plansys2::Predicate("(patrolled " + current_wp + ")"));
    problem_expert_->removePredicate(plansys2::Predicate("(is_shot " + current_wp + ")"));

    // Set the goal for next state
    problem_expert_->setGoal(plansys2::Goal("(and(patrolled " + next_wp + "))"));

    auto plan = computePlan();
    if (plan.has_value()) {
      std::cout << "Plan found for " << next_wp << std::endl;
      // We don't execute the plan, just move to the next state
      state_ = next_state;
    } else {
      std::cout << "Could not find plan for " << next_wp << std::endl;
      state_ = STOP;
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
