set(controller_SRC
  PepperFSMController.cpp
)

set(controller_HDR
  PepperFSMController.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DPepperFSMController_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)
target_link_libraries(${PROJECT_NAME} PUBLIC mc_pepper::constraints)
target_link_libraries(${PROJECT_NAME} PUBLIC mc_pepper::devices)
target_link_libraries(${PROJECT_NAME} PUBLIC mc_pepper::tasks)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)
