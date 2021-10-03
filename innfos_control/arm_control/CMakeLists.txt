add_library(WzSerialPort src/WzSerialPort.cpp)
target_link_libraries(WzSerialPort ${catkin_LIBRARIES})
add_dependencies(WzSerialPort ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(Demo3 pick_item_demo)
add_executable(${Demo3} src/${Demo3}.cpp)
target_link_libraries(${Demo3} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Demo3} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(Demo4 open_door_demo)
add_executable(${Demo4} src/${Demo4}.cpp)
target_link_libraries(${Demo4} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Demo4} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(Demo5 plug_demo)
add_executable(${Demo5} src/${Demo5}.cpp)
target_link_libraries(${Demo5} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Demo5} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(Demo6 robot_state_test)
add_executable(${Demo6} src/${Demo6}.cpp)
target_link_libraries(${Demo6} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Demo6} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(Demo7 open_door_demo_v2)
add_executable(${Demo7} src/${Demo7}.cpp)
target_link_libraries(${Demo7} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${Demo7} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(gripper_control src/gripper_control.cpp)
target_link_libraries(gripper_control ${catkin_LIBRARIES} ${Boost_LIBRARIES} WzSerialPort)
install(TARGETS gripper_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
