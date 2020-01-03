# CMake generated Testfile for 
# Source directory: /home/mrjohd/MotionPlanning_ws/src/DynamicMotionPlanning/xacro-melodic-devel
# Build directory: /home/mrjohd/MotionPlanning_ws/src/cmake-build-debug/DynamicMotionPlanning/xacro-melodic-devel
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_xacro_nosetests_test "/home/mrjohd/MotionPlanning_ws/src/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/mrjohd/MotionPlanning_ws/src/cmake-build-debug/test_results/xacro/nosetests-test.xml" "--return-code" "\"/home/mrjohd/clion/bin/cmake/linux/bin/cmake\" -E make_directory /home/mrjohd/MotionPlanning_ws/src/cmake-build-debug/test_results/xacro" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/mrjohd/MotionPlanning_ws/src/DynamicMotionPlanning/xacro-melodic-devel/test --with-xunit --xunit-file=/home/mrjohd/MotionPlanning_ws/src/cmake-build-debug/test_results/xacro/nosetests-test.xml")
set_tests_properties(_ctest_xacro_nosetests_test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;143;add_test;/opt/ros/melodic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/mrjohd/MotionPlanning_ws/src/DynamicMotionPlanning/xacro-melodic-devel/CMakeLists.txt;25;catkin_add_nosetests;/home/mrjohd/MotionPlanning_ws/src/DynamicMotionPlanning/xacro-melodic-devel/CMakeLists.txt;0;")
subdirs("test")
