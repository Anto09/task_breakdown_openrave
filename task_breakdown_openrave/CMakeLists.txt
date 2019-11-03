cmake_minimum_required(VERSION 2.8.3)
project(task_breakdown_openrave)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roscpp_serialization
    std_msgs
    roscpp
    rospy
    sdf_tools
    std_msgs
    geometry_msgs
    message_generation
    baxter_core_msgs
    baxter_examples
    baxter_interface
    baxter_maintenance_msgs
    baxter_tools
    interactive_markers
    moveit_core
    moveit_ros_perception
    moveit_ros_planning_interface
    pluginlib
    moveit_ros_manipulation

)

find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(LAPACK REQUIRED)
find_package(OpenRAVE 0.8 REQUIRED)
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

catkin_python_setup()

add_message_files(
    FILES
)

add_service_files(
    FILES
    OpenRaveIK.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    trajectory_msgs
    moveit_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}  add_two_ints _add_two_ints_wrapper_cpp
        CATKIN_DEPENDS sdf_tools roscpp rospy std_msgs moveit_core moveit_ros_planning_interface baxter_core_msgs baxter_examples pluginlib
        DEPENDS system_lib Eigen
)

## Set build flags for working with OpenMP
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    SYSTEM)
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
install(TARGETS ${IKFAST_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

add_library(add_two_ints
            include/task_breakdown_openrave/add_two_ints.h
            src/add_two_ints.cpp)
add_library(_add_two_ints_wrapper_cpp
            src/add_two_ints_wrapper.cpp)

target_link_libraries(add_two_ints ${catkin_LIBRARIES})
target_link_libraries(_add_two_ints_wrapper_cpp add_two_ints ${catkin_LIBRARIES} ${Boost_LIBRARIES})

set_target_properties(_add_two_ints_wrapper_cpp PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )


link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

set_target_properties(task_breakdown PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
