This work is based on Dr. Chen Feng's **[AHC]** algorithm. If you use this work, please check and follow the instruction below.

---------------
[AHC]: http://www.merl.com/research/?research=license-request&sw=PEAC

Fast Plane Extraction Using Agglomerative Hierarchical Clustering (AHC)
=======================================================================

Legal Remarks
-------------
Copyright 2014 Mitsubishi Electric Research Laboratories All
Rights Reserved.

Permission to use, copy and modify this software and its
documentation without fee for educational, research and non-profit
purposes, is hereby granted, provided that the above copyright
notice, this paragraph, and the following three paragraphs appear
in all copies.

To request permission to incorporate this software into commercial
products contact: Director; Mitsubishi Electric Research
Laboratories (MERL); 201 Broadway; Cambridge, MA 02139.

IN NO EVENT SHALL MERL BE LIABLE TO ANY PARTY FOR DIRECT,
INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
DOCUMENTATION, EVEN IF MERL HAS BEEN ADVISED OF THE POSSIBILITY OF
SUCH DAMAGES.

MERL SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
"AS IS" BASIS, AND MERL HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE,
SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

-------------
The difference from original source code is following.
- ~~Update OpenNI version to 2.2~~
- Fix some points to compile successfully
- Apply to RealSense
- Add some changes to use in ROS environment

My environment is 
- Ubuntu 16.04.4 LTS
- Linux Kernel 4.4.0-130-generic
- GCC 5.4.0
- PCL 1.8.1
- OpenCV 2.4.13 or 3.1.0
- ROS kinetic

Usage for ROS
-------------
Move into your catkin workspace,  
`$ cd PATH_TO_YOUR_WORKSPACE/src`

Download realsense package to your workspace if you did not, following [here](https://github.com/intel-ros/realsense)

Download this PEAC package into your workspace


`$ catkin_make`

Start the realsense node  
`$ roslaunch realsense2_camera rs_rgbd.launch`

Bring up windows  
`$ rosrun peac plane_fitter_ros input:=/camera/depth_registered/points`