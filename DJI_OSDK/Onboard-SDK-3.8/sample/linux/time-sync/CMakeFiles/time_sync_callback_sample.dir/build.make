# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8

# Include any dependencies generated for this target.
include sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/depend.make

# Include the progress variables for this target.
include sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/progress.make

# Include the compile flags for this target's objects.
include sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/flags.make

sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.o: sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/flags.make
sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.o: sample/linux/time-sync/time_sync_callback_sample.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.o"
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.o -c /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync/time_sync_callback_sample.cpp

sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.i"
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync/time_sync_callback_sample.cpp > CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.i

sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.s"
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync/time_sync_callback_sample.cpp -o CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.s

sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.o: sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/flags.make
sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.o: sample/linux/common/dji_linux_environment.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.o"
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.o -c /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/common/dji_linux_environment.cpp

sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.i"
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/common/dji_linux_environment.cpp > CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.i

sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.s"
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/common/dji_linux_environment.cpp -o CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.s

sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.o: sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/flags.make
sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.o: sample/linux/common/dji_linux_helpers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.o"
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.o -c /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/common/dji_linux_helpers.cpp

sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.i"
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/common/dji_linux_helpers.cpp > CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.i

sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.s"
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/common/dji_linux_helpers.cpp -o CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.s

# Object files for target time_sync_callback_sample
time_sync_callback_sample_OBJECTS = \
"CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.o" \
"CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.o" \
"CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.o"

# External object files for target time_sync_callback_sample
time_sync_callback_sample_EXTERNAL_OBJECTS =

build/bin/time_sync_callback_sample: sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/time_sync_callback_sample.cpp.o
build/bin/time_sync_callback_sample: sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_environment.cpp.o
build/bin/time_sync_callback_sample: sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/__/common/dji_linux_helpers.cpp.o
build/bin/time_sync_callback_sample: sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/build.make
build/bin/time_sync_callback_sample: libs/libdjiosdk-core.a
build/bin/time_sync_callback_sample: sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../../../build/bin/time_sync_callback_sample"
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/time_sync_callback_sample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/build: build/bin/time_sync_callback_sample

.PHONY : sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/build

sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/clean:
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync && $(CMAKE_COMMAND) -P CMakeFiles/time_sync_callback_sample.dir/cmake_clean.cmake
.PHONY : sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/clean

sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/depend:
	cd /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8 /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8 /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync /home/local/MBZIRC2023/DJI_OSDK/Onboard-SDK-3.8/sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sample/linux/time-sync/CMakeFiles/time_sync_callback_sample.dir/depend

