# ros_external_validator

A ROS module to synchronize ros tests executions with external V&V tool (ej.: ULISES) via ROS-MQTT bridge.

The aim of this project is to communicate test runs in ROS with the ULISES diagnostic tool. The communication protocol/system to be created will be tested with ULISES but in the future it could be used to communicate other validation tools with ROS test runs.


ULISES currently receives positioning (tf) data via ROS-MQTT bridge. The idea is that ULISES not only receives data to be analysed but also messages indicating which test is to be performed, and that the test has started. In turn, ULISES will indicate to the ROS test whether the system has behaved correctly or not. This information could be used as ASSERT in the tests.

The basic functionalities to be implemented will be:

1. TEST_START message
2. TEST_STATUS / ASSERT message
3. TEST END message
4. TEST ERROR message

The next step is to set up a meeting to define how this work will be done. 
- Definition of message interfaces
- Use case/Simulation with which to test this coordinator
- Define the programming language Pyhton vs C/C++
  
  