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
CMAKE_SOURCE_DIR = /home/local/MBZIRC2023/CameraServer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/local/MBZIRC2023/CameraServer/build

# Include any dependencies generated for this target.
include CMakeFiles/boxDetection2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/boxDetection2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/boxDetection2.dir/flags.make

CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.o: CMakeFiles/boxDetection2.dir/flags.make
CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.o: ../Jacob_box/BoxDetection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/MBZIRC2023/CameraServer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.o -c /home/local/MBZIRC2023/CameraServer/Jacob_box/BoxDetection.cpp

CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/MBZIRC2023/CameraServer/Jacob_box/BoxDetection.cpp > CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.i

CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/MBZIRC2023/CameraServer/Jacob_box/BoxDetection.cpp -o CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.s

# Object files for target boxDetection2
boxDetection2_OBJECTS = \
"CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.o"

# External object files for target boxDetection2
boxDetection2_EXTERNAL_OBJECTS =

libboxDetection2.a: CMakeFiles/boxDetection2.dir/Jacob_box/BoxDetection.cpp.o
libboxDetection2.a: CMakeFiles/boxDetection2.dir/build.make
libboxDetection2.a: CMakeFiles/boxDetection2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/local/MBZIRC2023/CameraServer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libboxDetection2.a"
	$(CMAKE_COMMAND) -P CMakeFiles/boxDetection2.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/boxDetection2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/boxDetection2.dir/build: libboxDetection2.a

.PHONY : CMakeFiles/boxDetection2.dir/build

CMakeFiles/boxDetection2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/boxDetection2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/boxDetection2.dir/clean

CMakeFiles/boxDetection2.dir/depend:
	cd /home/local/MBZIRC2023/CameraServer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/local/MBZIRC2023/CameraServer /home/local/MBZIRC2023/CameraServer /home/local/MBZIRC2023/CameraServer/build /home/local/MBZIRC2023/CameraServer/build /home/local/MBZIRC2023/CameraServer/build/CMakeFiles/boxDetection2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/boxDetection2.dir/depend
