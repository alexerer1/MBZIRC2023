# Dynamixel Test program (without catkin / ROS)

## Dependencies

Install dependencies:

```bash
sudo apt-get install libboost-system-dev build-essential cmake
```

## Generate the build files from the [CMakeLists.txt](./CMakeLists.txt)

```bash
cmake .
```

## Build the executable

```bash
make
```

## Run the executable

Note: Please look at [main.cpp](./main.cpp) for all the options and to
see the example code for how to use the dynamixel library.

```bash
./dynamixel_test --portName <serial port> --baudRate <baud rate> --motorId <motor id>
```

e.g.

```bash
./dynamixel_test --portName /dev/ttyUSB0 --baudRate 9600 --motorId 1
```

