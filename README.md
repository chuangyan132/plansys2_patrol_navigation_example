# PlanSys2 Patrol Navigation Example

## Description

(**Requires Navigation2, warehouse project in isaac simulation**)

This example shows:
- How to use an application to initiate knowledge (the PDDL problem) and control its execution flow.
- How to interact with ROS2 Navigation

The patrolling_controller_node node must be run separately. Use the PlanSys2 API to populate instances, predicates, and goals. Implement a finite state machine to establish as a goal which waypoint to patrol. Patrolling a waypoint is going to that point and turning around for a few seconds.

## How to run

In terminal 1:

```
ros2 launch plansys2_patrol_navigation_example patrol_example_launch.py
```

In terminal 2:

```
ros2 run plansys2_patrol_navigation_example patrolling_controller_node
```

## Explanation
### What is this package?
- This package is a planner and executor to manage the specific workflow in a scenario: Warehouse simulated in Isaac Sim
- Planner: it reads domain.pddl into plansys2, and problem into problem.expert. Then it translate the plan generated from pddl by pddl planner into a BT file. The executor(controller node in src) will read the BT file and call actions which run at behind by launch file in specific sequence controlled by BT file.
- pddl
  - different domain.pddl to describe different case in scenario warehouse.
- src
  - skill ros2 node
    - ask charge
    - charging
    - move
    - patrol
    - shot
  - controller ros2 node
    - patrolling_controller_node.cpp
    - patrolling_controller_test1_03_node.cpp
    - ...
- tasks
  - used in controller ros node. describe a task used in controller node. the controller node will read it and parse it to class itself.
### How does it work?
1. Define first domain.pddl
   1. Define types
   2. Define predicates
   3. Define action(action name should be same as ros2 node)
2. Define problem
   1. Define Instances
   2. Define Predicates
   3. Define the Goal(Goal is also predicates)
3. Create ROS2 node in src
   1. controller node 
      1. We can define the problem here
      ``` cpp
      //initialization
      init() 
      // add problem in pddl to controller.(instances, predicates, goal)
      init_knowledge() 
      // step() manage different state of workflow, 
      step()
      ```
   2. skil node
      1. even you have ros2 node coded already, you still need to write the node inherit from plansys2, so the controller can manage it. but you can create the skil node as client to call service from your skil node.
      2. name in skil node should be same as name in pddl action.
      ``` cpp
      node->set_parameter(rclcpp::Parameter("action_name", "askcharge"));
      //name
      node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      //the skill node will run always, the lifecycle is managed by controller node.
      ```
4. RUN
   1. prepare two terminal, one is launch plansys2_bringup. second is run the controller node in src
   2. The BT and generated plan is in hide mode, not visible to user. We can print the progress in step function inside the file: controller_node.cpp

### How to use plansys2 to solve a task?
1. Define a domain and problem as pddl format for your task. The goal is described as predicate.
2. Write skill node and controller node inherited from plansys2.
3. RUN
   1. launch plansys2_bringup 
   2. run controller_node.cpp
### Dive into code
future work
## Comments
