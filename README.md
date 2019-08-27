# amcl3d

### Overview

This is a package is a **"Adaptive Monte-Carlo Localization in 3D"**.

It is a particle filter that estimates the localization of a robot moving in a 3D environment without using GPS.

It takes information from an odometry source, point-clouds from an onboard sensor (e.g. laser) and distance measurements from radio-range sensors.

#### License

Apache 2.0

**Author: Paloma Carrasco Fernández (pcarrasco@catec.aero),
          Francisco Cuesta Rodríguez (fcuesta@catec.aero),
          Francisco J.Perez-Grau (fjperez@catec.aero)**

**Affiliation: [FADA-CATEC](https://http://www.catec.aero//)**

**Maintainer: Paloma Carrasco Fernández (pcarrasco@catec.aero),
              Francisco Cuesta Rodríguez (fcuesta@catec.aero)**

The amcl3d package has been tested under [ROS] Kinetic and Ubuntu 16.04.

#### Publications

If you want more information about the algorithm or use this work in your project, please check and cite the following publication:

* Francisco J.Perez-Grau, Fernando Caballero, Antidio Viguria and Anibal Ollero:

	**[Multi-sensor 3D Monte Carlo Localization (MCL) for long-term aerial robot navigation, 2017](https://journals.sagepub.com/doi/pdf/10.1177/1729881417732757)**

#### Detailed Description

To know in more detail the behavior of the package:

* **[amcl3d (Wiki-ROS)](http://wiki.ros.org/amcl3d#preview)**

### Installation

#### Building from Source

##### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

     cd catkin_workspace/src
     git clone ...
     cd ../
     catkin build

### Tests

Run the test with

     roslaunch ouster_ros os1.launch os1_hostname:=10.5.5.94 replay:=true
     roslaunch amcl3d amcl3d_test.launch

### Launch files

* **amcl3d.launch:** it contains the start of amcl3d node with a standard configuration of parameters.

          roslaunch amcl3d amcl3d.launch
		  
* **amcl3d_test.launch:**  this roslaunch allows you to start the RViz with the aforementioned configuration, the amcl3d node, the test-amcl3d node, the bag player and creates a transformation to relate the point-cloud frame of test-amcl3d node with the robot frame of amcl3d node.
          
		  roslaunch amcl3d amcl3d_test.launch

### Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/fada-catec/amcl3d/issues).

### Acknowledgement

![ROSIN](http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png)


Supported by ROSIN - ROS-Industrial Focused Technical Projects (FTP).  
More information: [rosin-project.eu](http://rosin-project.eu)
