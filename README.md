# dyros_red_sim

* This project is controller of humanoid robot DYROS RED, which is torque controlled robot. It's main control algorithm is based on "Contact Consistent Control for Humanoid Robots, Jaeheung Park".
This controller can be simulated through [mujoco_ros](https://github.com/saga0619/mujoco_ros_sim)

## Environment 
* Tested by ubuntu 16.04 , 18.04
* for mujoco users, 18.04 is better than 16.04 since there are graphical issues in 16.04
* faster communication between mujoco and controller at ubuntu 18.04 (lag of mujoco or controller due to graphical problem?? 18.04 is faster anyway :) )
* there are qpOASES installation problem at 18.04, can be solved with adding compile option

## SOEM Setup
 * install [SOEM](https://github.com/saga0619/SOEM)

## RBDL Setup 
### Installing
```sh
wget https://bitbucket.org/rbdl/rbdl/get/default.zip
unzip default.zip
cd rbdl-rbdl-0879ee8c548a
mkdir build
cd build
cmake -D RBDL_BUILD_ADDON_URDFREADER=ON ..
make all
sudo make install
```

### RBDL Error handling
* If any error related to ROS occurs, open rbdl-rbdl-[commit]/addons/urdfreader/urdfreader.cc and remove following line
```cpp
#include <ros.h>
```
* If error " 'boost' does not name a type" occurs, open rbdl-rbdl-[commit]/addons/urdfreader/urdfreader.cc and edit boost::shared_ptr to std::shared_ptr. (line 15~18)
```cpp
typedef std::shared_ptr<urdf::Link> LinkPtr;
typedef const std::shared_ptr<const urdf::Link> ConstLinkPtr;
typedef std::shared_ptr<urdf::Joint> JointPtr;
typedef std::shared_ptr<urdf::ModelInterface> ModelPtr;
```

* If red controller can't find librbdl.so.2.6.0, Add following line to .bashrc 
```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```


## qpOASES setup
Download qpOASES [Link](http://www.qpoases.org/go/release) 
```sh
cd qpOASES-3.2.1
mkdir build
cd build
cmake ..
make all
sudo make install
```

### qpOASES error handling
if error occures, add following line to qpOASES-3.2.1/CMakeLists.txt, below PROJECT(qpOASES CXX), which is line 34

```
add_compile_options(-fPIC)
```

## Dyros Red Controller Setup
Download 1.0 version from release list. [1.0 release](https://github.com/saga0619/dyros_red/releases)
Unzip at ros worksapce (ex:catkin_ws)

## ncurse Setup
```sh 
sudo apt-get install libncurses5-dev
```

## Mujoco setup

### license file 
For mujoco user, you need to edit the position of license in dyros_red_simulation/simulation.launch with your license file's location
(mjkey.txt at Home for default)

### How to start with mujoco ###
clone mujoco_ros_sim, then build. 
* git clone https://github.com/saga702/mujoco_ros_sim

after build, launch simulation.launch 
```sh
roslaunch dyros_red_launch simulation.launch 
```
