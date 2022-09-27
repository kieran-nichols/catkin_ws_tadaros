set(SOURCES simple_test.cpp)
add_executable(simple_test ${SOURCES})
target_link_libraries(simple_test soem ${catkin_LIBRARIES})
install(TARGETS simple_test DESTINATION bin)
