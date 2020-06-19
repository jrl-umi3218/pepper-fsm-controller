cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME PepperFSMController)
set(PROJECT_DESCRIPTION "PepperFSMController")
set(PROJECT_URL "https://gite.lirmm.fr/mc-controllers/pepperfsmcontroller")

project(${PROJECT_NAME} CXX)

find_package(mc_rtc 1.4.0  REQUIRED)
find_package(mc_pepper REQUIRED)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/PepperFSMController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")
