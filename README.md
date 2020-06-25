# amcl3d

[![Build Status](https://travis-ci.org/fada-catec/amcl3d.svg?branch=master)](https://travis-ci.org/fada-catec/amcl3d)
[![License](https://img.shields.io/badge/License-Apache%202-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![codecov](https://codecov.io/gh/fada-catec/amcl3d/branch/kinetic-test/graph/badge.svg)](https://codecov.io/gh/fada-catec/amcl3d)

### Overview

This is a package is a **"Adaptive Monte-Carlo Localization in 3D"**.

It is a particle filter that estimates the localization of a robot moving in a 3D environment without using GPS.

It takes information from an odometry source, point-clouds from an onboard sensor (e.g. laser) and distance measurements from radio-range sensors.

#### License

Apache 2.0

**Author: Paloma Carrasco Fern�ndez (pcarrasco@catec.aero),
          Francisco Cuesta Rodr�guez (fcuesta@catec.aero),
          Francisco J.Perez-Grau (fjperez@catec.aero)**

**Affiliation: [FADA-CATEC](https://http://www.catec.aero//)**

**Maintainer: Paloma Carrasco Fern�ndez (pcarrasco@catec.aero),
              Francisco Cuesta Rodr�guez (fcuesta@catec.aero)**

The amcl3d package has been tested under [ROS] Kinetic and Ubuntu 16.04.

#### Publications

If you want more information about the algorithm or use this work in your project, please check and cite the following publication:

* Francisco J.Perez-Grau, Fernando Caballero, Antidio Viguria and Anibal Ollero:

	**[Multi-sensor 3D Monte Carlo Localization (MCL) for long-term aerial robot navigation, 2017](https://journals.sagepub.com/doi/pdf/10.1177/1729881417732757)**

#### Detailed Description

To know in more detail the behavior of the package:

* **[amcl3d (Wiki-ROS)](http://wiki.ros.org/amcl3d#preview)**

### Demostration Video

[![](http://img.youtube.com/vi/Dn6LxH-WLRA/0.jpg)](http://www.youtube.com/watch?v=Dn6LxH-WLRA "")

### Installation

#### Building from Source

##### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

     cd catkin_workspace/src
     git clone https://github.com/fada-catec/amcl3d.git
     cd ../
     catkin build

### Launch files

* **amcl3d.launch:** it contains the start of amcl3d node with a standard configuration of parameters.

          roslaunch amcl3d amcl3d.launch
		  
* **amcl3d_rosin.launch:**  it contains the initial pose, particle number, 'alpha' parameter, 'take_off_height' parameter and the correctly map to run the algorithm with the correcly data of the rosbag. 
          
		roslaunch amcl3d amcl3d_rosin.launch

### Gmoke Tests

This branch contains differents tests to evaluate the correct behaviour of the algorithm. To run these tests it is necessary to have the rosbag of the 'Version 1.1.0' release.

     To compile:

          catkin_make tests

     To run:

          rosrun amcl3d amcl3d_tests


### Doxygen
The code has been commentes to offert the posibility to generate a Doxygen documentation. To generate it:

     rosdoc_lite /path/to/workspace/src/amcl3d/amcl3d

To install the rosdoc_lite package:

     apt-get install ros-kinetic-rosdoc_lite
     
### Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/fada-catec/amcl3d/issues).

### Acknowledgement

![ROSIN](http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png)


Supported by ROSIN - ROS-Industrial Focused Technical Projects (FTP).  
More information: [rosin-project.eu](http://rosin-project.eu)
