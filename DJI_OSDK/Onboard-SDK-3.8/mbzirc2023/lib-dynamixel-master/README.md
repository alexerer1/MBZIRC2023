Dynamixel AX-12A C++ Library
============================

Install Build Tools
-------------------------

```bash
sudo apt-get install python-catkin-tools
```

Build Library:
-------------

```bash
cd
mkdir catkin_ws #if a workspace does not already exist
cd catkin_ws
git clone https://github.com/rosmod/lib-dynamixel.git src/lib-dynamixel
catkin build dynamixel
```

Update Library:
-----------------

```bash
cd ~/catkin_ws
catkin clean dynamixel
cd src/lib-dynamixel
git pull
cd ..
catkin build dynamixel
```

# Rosmod source library setup

1. In this github repo navigate to [releases](https://github.com/rosmod/lib-dynamixel/releases), right click on `pid.zip` (not the source code zip!) and select `Copy link address'
2. In a rosmod project, drag in a new source library to the software model
3. Paste the link in the url attribute
4. Name the source library `dynamixel`
5. Drag the library into the `set editor` of any component that uses it
6. In the forwards section of the component add `#include "dynamixel.h`


Use the Library without ROS:
----------------------------

You will find an example [CMake File](./test/CMakeLists.txt) and
[source file](./test/main.cpp).  These are all found within the
[test](./test) directory, which contains a README as well. This serves
as an example of how to integrate this code into a project that does
not use ROS.
