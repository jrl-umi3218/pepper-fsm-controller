cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME PepperFSMController)

project(${PROJECT_NAME} CXX)

find_package(mc_rtc 1.4.0 REQUIRED)
find_package(mc_pepper REQUIRED)
find_package(mc_human REQUIRED)

add_subdirectory(src)

set(CONF_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml")
configure_file(etc/PepperFSMController.in.yaml "${CONF_OUT}")
install(FILES "${CONF_OUT}" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")
