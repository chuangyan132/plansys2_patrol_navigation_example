cmake_minimum_required(VERSION 3.5)
project(plansys2_patrol_navigation_example)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    nav2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    sensor_msgs
    cv_bridge
    OpenCV
    ament_index_cpp
)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(patrol_action_node src/patrol_action_node.cpp)
ament_target_dependencies(patrol_action_node ${dependencies})

add_executable(patrolling_controller_node src/patrolling_controller_node.cpp)
ament_target_dependencies(patrolling_controller_node ${dependencies})

add_executable(patrolling_controller_test1_03_node src/patrolling_controller_test1_03_node.cpp)
ament_target_dependencies(patrolling_controller_test1_03_node ${dependencies})



target_link_libraries(
  patrolling_controller_test1_03_node 
  ${ament_index_cpp_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

target_include_directories(
  patrolling_controller_test1_03_node
  PRIVATE
  ${ament_index_cpp_LIBRARIES}
  ${YAML_CPP_INCLUDE_DIRS}
  )



add_executable(patrolling_controller_test1_04_node src/patrolling_controller_test1_04_node.cpp)
ament_target_dependencies(patrolling_controller_test1_04_node ${dependencies})
target_link_libraries(
  patrolling_controller_test1_04_node 
  ${ament_index_cpp_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

target_include_directories(
  patrolling_controller_test1_04_node
  PRIVATE
  ${ament_index_cpp_LIBRARIES}
  ${YAML_CPP_INCLUDE_DIRS}
  )

add_executable(patrolling_controller_test2_01_node src/patrolling_controller_test2_01_node.cpp)
ament_target_dependencies(patrolling_controller_test2_01_node ${dependencies})
target_link_libraries(
  patrolling_controller_test2_01_node 
  ${ament_index_cpp_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
  
  target_include_directories(
    patrolling_controller_test2_01_node
    PRIVATE
    ${ament_index_cpp_LIBRARIES}
    ${YAML_CPP_INCLUDE_DIRS}
    )


add_executable(ask_charge_action_node src/ask_charge_action_node.cpp)
ament_target_dependencies(ask_charge_action_node ${dependencies})

add_executable(charge_action_node src/charge_action_node.cpp)
ament_target_dependencies(charge_action_node ${dependencies})

add_executable(shot_action_node src/shot_action_node.cpp)
ament_target_dependencies(shot_action_node ${dependencies})

install(DIRECTORY launch pddl params tasks DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  patrol_action_node
  patrolling_controller_node
  patrolling_controller_test1_03_node
  patrolling_controller_test1_04_node
  patrolling_controller_test2_01_node
  ask_charge_action_node
  charge_action_node
  shot_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
