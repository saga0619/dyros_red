# dyros_red_sim

### Environment ###
* Tested at ubuntu 16.04 , 18.04
* for mujoco users, 18.04 is better than 16.04 since there are graphical issues in 16.04
* faster communication between mujoco and controller at ubuntu 18.04 (lag of mujoco or controller due to graphical problem?? 18.04 is faster anyway :) )
* there are qpOASES installation problem at 18.04, can be solved with adding compile option


### RBDL Setup ###
```sh
wget https://bitbucket.org/rbdl/rbdl/get/default.zip
unzip default.zip
cd rbdl-rbdl-849d2aee8f4c
mkdir build
cd build
cmake -D RBDL_BUILD_ADDON_URDFREADER=ON ..
make all
sudo make install
```
* If an error occurs, open rbdl-rbdl-[commit]/addons/urdfreader/urdfreader.cc
* and remove this line
```cpp
#include <ros.h>
```
* If controller can't find librbdl.so.2.6.0, add following line to .bashrc
```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```


### Mujoco license file ###
* for mujoco user, you need to edit the position of license in dyros_red_simulation/simulation.launch with your license file's location

### How to start with mujoco ###
clone mujoco_ros_sim, then build. 
* git clone https://github.com/saga702/mujoco_ros_sim

after build, launch simulation.launch 
```sh
roslaunch dyros_red_launch simulation.launch 
```

### qpOASES install ###
* Download qpOASES from https://projects.coin-or.org/qpOASES/wiki/QpoasesDownload
```sh
cd qpOASES-3.2.1
mkdir build
cd build
cmake ..
make all
sudo make install
```

*if error occures, add following line to qpOASES-3.2.1/CMakeLists.txt, below PROJECT(qpOASES CXX), which is line 34

```
add_compile_options(-fPIC)
```
