# RTDE-URx
RTDE class to control a Universal Robot. This repo was modified to work as a minimal example alongside the simple ur5e sim.

This class is built on the RTDE package proposed by Universal Robots at https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/real-time-data-exchange-rtde-guide-22229/

A python script (RTDEclass.py) communciates with the docker container using the provided RTDE interface. They communicate throught registers defined in control_loop_configuration.xml.

Registers can be written/read by one script and read/written by the other.
* State (from UR to python)
  * Actual_TCP_pose : Actual TCP pose
  
* Setp (from python to UR)
  * Joint_1, Joint_2, Joint_3, Joint_4, Joint_5, Joint_6 : Target joint positions in Radians
