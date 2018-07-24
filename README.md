# dyros_red_sim



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
