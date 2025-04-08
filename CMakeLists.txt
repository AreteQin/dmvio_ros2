cmake_minimum_required(VERSION 3.8) # Recommended for ROS2
project(dmvio_ros2)

set(CMAKE_CXX_STANDARD 14) # Keep 14, or consider 17 if needed
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules) # Add elements to list

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# --- ROS2 Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(rosbag2_cpp) # Add if rosbag functionality is needed later
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED) # For message generation
find_package(std_msgs REQUIRED) # Dependency for Header in msg

# --- External Dependencies ---
# Keep CMAKE_MODULE_PATH if dm-vio/cmake_modules has custom find scripts needed
# list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread filesystem chrono serialization date_time timer regex REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Pangolin REQUIRED)

# Ensure librealsense2-dev is installed system-wide
#find_package(realsense2 REQUIRED)

# --- DM-VIO Core Library ---
# This assumes dm-vio itself doesn't have hardcoded ROS1 dependencies
# and provides a library target (e.g., 'dmvio')
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/dm-vio)
set(DMVIO_LIBRARY_TARGET dmvio) # Assume this target is created by dm-vio's CMakeLists

# --- Include Directories ---
include_directories(
        include # Local include directory
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIRS} # Check if GTSAM provides this variable or use target_include_directories
        ${GLOG_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
        # Pangolin includes might be handled by linking its target, or add explicitly:
        ${Pangolin_INCLUDE_DIRS}
        ${realsense2_INCLUDE_DIR} # Check variable name provided by Findrealsense2.cmake
        # DM-VIO specific includes
        ${PROJECT_SOURCE_DIR}/third_party/dm-vio/src
        ${PROJECT_SOURCE_DIR}/third_party/dm-vio/src/dso
        ${PROJECT_SOURCE_DIR}/third_party/dm-vio/thirdparty/Sophus
        # OpenCV includes usually handled by target_link_libraries(${OpenCV_LIBS})
)
# Note: Ament preferred way is target_include_directories after target definition

# --- Compiler Flags ---
# Keep these settings, they are general C++ flags
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
        "${SSE_FLAGS}"
)
set(CMAKE_CXX_FLAGS_DEBUG
        "-O2 -g -fno-omit-frame-pointer -DEIGEN_INITIALIZE_MATRICES_WITH_NAN -DDEBUG ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
        "-O3 -g -fno-omit-frame-pointer ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELEASE
        "-O3 -DNDEBUG"
)
IF (${Boost_VERSION} GREATER_EQUAL 106500)
    message(STATUS "Building with stacktrace support.")
    set(STACKTRACE_LIBRARIES dl)
    set(STACKTRACE_DEFINES "-DSTACKTRACE -DBOOST_STACKTRACE_USE_ADDR2LINE")
ENDIF ()

# --- Message Generation ---
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/DMVIOPoseMsg.msg"
        DEPENDENCIES geometry_msgs std_msgs # Add dependencies used within the msg file
)

# Export dependencies for downstream packages.
ament_export_dependencies(rosidl_default_runtime)

# --- Executable: dmvio_node ---
add_executable(dmvio_node src/main.cpp src/ROSOutputWrapper.cpp)
#add_dependencies(dmvio_node ${PROJECT_NAME}_generate_messages_cpp)
ament_target_dependencies(dmvio_node
        rclcpp
        cv_bridge
        geometry_msgs
        sensor_msgs
        image_transport
        tf2
        tf2_ros
        std_msgs # Added
        # Add ROS2 packages providing non-ament dependencies if available, e.g.
        # realsense2_camera # If using the ROS2 realsense node package
)
target_include_directories(dmvio_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include> # For installation
        # Add other necessary include paths here if not handled globally
        ${EIGEN3_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/third_party/dm-vio/src
        ${PROJECT_SOURCE_DIR}/third_party/dm-vio/src/dso
        ${PROJECT_SOURCE_DIR}/third_party/dm-vio/thirdparty/Sophus
        ${GLOG_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(dmvio_node
        # Link against the DM-VIO library target
        ${DMVIO_LIBRARY_TARGET}
        # Link external libraries
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${GTSAM_LIBRARIES} # Check variable name from find_package(GTSAM)
        ${GLOG_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${STACKTRACE_LIBRARIES}
        ${cpp_typesupport_target}
        fmt::fmt
        GLEW::GLEW
        ${OPENGL_LIBRARIES}
        gtsam
        ${Pangolin_LIBRARIES}# If using GTSAM as a target
)

# --- Executable: D435_publisher ---
# If this node is still needed, convert it similarly.
# Commenting out for now as it wasn't the primary focus.
# add_executable(D435_publisher src/D435_publisher.cpp)
# ament_target_dependencies(D435_publisher
#     rclcpp
#     cv_bridge
#     sensor_msgs
#     image_transport
#     realsense2 # Assuming this provides the necessary linkage info
#     ${OpenCV_LIBS} # OpenCV handled separately
# )
# target_link_libraries(D435_publisher
#     ${OpenCV_LIBS}
#     ${realsense2_LIBRARY} # Or link target realsense2::realsense2 if available
# )

# --- Install Rules ---
install(TARGETS
        dmvio_node
        # D435_publisher # If enabled
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

# Install other resources like launch files, config files if added
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/launch
# )
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/config
# )

# --- Finalize ---
ament_package() # Replaces catkin_package()