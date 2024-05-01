cmake_minimum_required(VERSION 3.8)
project(manus_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fmt REQUIRED)

find_package( PkgConfig REQUIRED )
pkg_check_modules ( ncurses++ REQUIRED ncurses++ )


# locate the MANUS SDK in the /ext folder
file(GLOB MANUS_SDK RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "ext/MANUS_Core_*")
if (NOT MANUS_SDK)
  message(FATAL_ERROR "MANUS SDK not found. Please download the SDK and install into the /ext folder.")
endif()

# Locate the SDKMinimalClient_Linux folder in the SDK path
file(GLOB MANUS_LINUX_PATH RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MANUS_SDK}/*/*/SDKMinimalClient_Linux")
if (NOT MANUS_LINUX_PATH)
  message(FATAL_ERROR "SDKMinimalClient_Linux folder not found. Please download the SDK and install into the /ext folder.")
endif()


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include directories
include_directories(
    include
    ${MANUS_LINUX_PATH}/ManusSDK/include
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
)


add_executable(manus_ros2 
  src/SDKMinimalClient.cpp
  src/manus_math.cpp
  src/manus_ros2.cpp
  )

# Specify the directory containing the shared library
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${MANUS_LINUX_PATH}/ManusSDK/lib)
set(LIBRARY_FILE ${LIBRARY_DIR}/libManusSDK.so)

# Link the Manus SDK library and other dependencies
target_link_libraries(manus_ros2
    PRIVATE
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    fmt
    ncurses
    ${LIBRARY_FILE} # Link the library
)

target_include_directories(manus_ros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(manus_ros2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS manus_ros2
  DESTINATION lib/${PROJECT_NAME})

# Install the MANUS library SO file
install(FILES ${LIBRARY_FILE} DESTINATION lib/${PROJECT_NAME})

# Embed the library into the executable
set_target_properties(manus_ros2 PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
