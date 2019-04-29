cmake_minimum_required(VERSION 3.5)
project(tf2_kdl)

if(NOT WIN32)
  add_definitions(-std=c++14)
endif()

find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

install(PROGRAMS scripts/test.py
         DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_kdl test/test_tf2_kdl.cpp)

	target_include_directories(test_kdl PUBLIC
    include
		${tf2_INCLUDE_DIRS}
		${tf2_ros_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
		${rclcpp_INCLUDE_DIRS}
		${orocos_kdl_INCLUDE_DIRS}
	)

  target_link_libraries(test_kdl
		${rclcpp_LIBRARIES}
		${tf2_LIBRARIES}
		${tf2_ros_LIBRARIES}
		${orocos_kdl_LIBRARIES}
	)
endif()

ament_export_include_directories(include ${Eigen3_INCLUDE_DIRS})
ament_export_dependencies(Eigen3 orocos_kdl)
ament_package()
