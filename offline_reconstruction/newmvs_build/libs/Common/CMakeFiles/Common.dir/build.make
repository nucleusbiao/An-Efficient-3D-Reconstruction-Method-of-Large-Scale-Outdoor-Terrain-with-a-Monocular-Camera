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
CMAKE_SOURCE_DIR = /home/zjd/3dpart/openmvs1.1.1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zjd/3dpart/openmvs1.1.1/openmvs_build

# Include any dependencies generated for this target.
include libs/Common/CMakeFiles/Common.dir/depend.make

# Include the progress variables for this target.
include libs/Common/CMakeFiles/Common.dir/progress.make

# Include the compile flags for this target's objects.
include libs/Common/CMakeFiles/Common.dir/flags.make

libs/Common/CMakeFiles/Common.dir/Common.cpp.o: libs/Common/CMakeFiles/Common.dir/flags.make
libs/Common/CMakeFiles/Common.dir/Common.cpp.o: ../libs/Common/Common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zjd/3dpart/openmvs1.1.1/openmvs_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libs/Common/CMakeFiles/Common.dir/Common.cpp.o"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Common.dir/Common.cpp.o -c /home/zjd/3dpart/openmvs1.1.1/libs/Common/Common.cpp

libs/Common/CMakeFiles/Common.dir/Common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Common.dir/Common.cpp.i"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zjd/3dpart/openmvs1.1.1/libs/Common/Common.cpp > CMakeFiles/Common.dir/Common.cpp.i

libs/Common/CMakeFiles/Common.dir/Common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Common.dir/Common.cpp.s"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zjd/3dpart/openmvs1.1.1/libs/Common/Common.cpp -o CMakeFiles/Common.dir/Common.cpp.s

libs/Common/CMakeFiles/Common.dir/CUDA.cpp.o: libs/Common/CMakeFiles/Common.dir/flags.make
libs/Common/CMakeFiles/Common.dir/CUDA.cpp.o: ../libs/Common/CUDA.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zjd/3dpart/openmvs1.1.1/openmvs_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object libs/Common/CMakeFiles/Common.dir/CUDA.cpp.o"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Common.dir/CUDA.cpp.o -c /home/zjd/3dpart/openmvs1.1.1/libs/Common/CUDA.cpp

libs/Common/CMakeFiles/Common.dir/CUDA.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Common.dir/CUDA.cpp.i"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zjd/3dpart/openmvs1.1.1/libs/Common/CUDA.cpp > CMakeFiles/Common.dir/CUDA.cpp.i

libs/Common/CMakeFiles/Common.dir/CUDA.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Common.dir/CUDA.cpp.s"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zjd/3dpart/openmvs1.1.1/libs/Common/CUDA.cpp -o CMakeFiles/Common.dir/CUDA.cpp.s

libs/Common/CMakeFiles/Common.dir/ConfigTable.cpp.o: libs/Common/CMakeFiles/Common.dir/flags.make
libs/Common/CMakeFiles/Common.dir/ConfigTable.cpp.o: ../libs/Common/ConfigTable.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zjd/3dpart/openmvs1.1.1/openmvs_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object libs/Common/CMakeFiles/Common.dir/ConfigTable.cpp.o"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Common.dir/ConfigTable.cpp.o -c /home/zjd/3dpart/openmvs1.1.1/libs/Common/ConfigTable.cpp

libs/Common/CMakeFiles/Common.dir/ConfigTable.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Common.dir/ConfigTable.cpp.i"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zjd/3dpart/openmvs1.1.1/libs/Common/ConfigTable.cpp > CMakeFiles/Common.dir/ConfigTable.cpp.i

libs/Common/CMakeFiles/Common.dir/ConfigTable.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Common.dir/ConfigTable.cpp.s"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zjd/3dpart/openmvs1.1.1/libs/Common/ConfigTable.cpp -o CMakeFiles/Common.dir/ConfigTable.cpp.s

libs/Common/CMakeFiles/Common.dir/EventQueue.cpp.o: libs/Common/CMakeFiles/Common.dir/flags.make
libs/Common/CMakeFiles/Common.dir/EventQueue.cpp.o: ../libs/Common/EventQueue.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zjd/3dpart/openmvs1.1.1/openmvs_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object libs/Common/CMakeFiles/Common.dir/EventQueue.cpp.o"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Common.dir/EventQueue.cpp.o -c /home/zjd/3dpart/openmvs1.1.1/libs/Common/EventQueue.cpp

libs/Common/CMakeFiles/Common.dir/EventQueue.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Common.dir/EventQueue.cpp.i"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zjd/3dpart/openmvs1.1.1/libs/Common/EventQueue.cpp > CMakeFiles/Common.dir/EventQueue.cpp.i

libs/Common/CMakeFiles/Common.dir/EventQueue.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Common.dir/EventQueue.cpp.s"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zjd/3dpart/openmvs1.1.1/libs/Common/EventQueue.cpp -o CMakeFiles/Common.dir/EventQueue.cpp.s

libs/Common/CMakeFiles/Common.dir/Log.cpp.o: libs/Common/CMakeFiles/Common.dir/flags.make
libs/Common/CMakeFiles/Common.dir/Log.cpp.o: ../libs/Common/Log.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zjd/3dpart/openmvs1.1.1/openmvs_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object libs/Common/CMakeFiles/Common.dir/Log.cpp.o"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Common.dir/Log.cpp.o -c /home/zjd/3dpart/openmvs1.1.1/libs/Common/Log.cpp

libs/Common/CMakeFiles/Common.dir/Log.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Common.dir/Log.cpp.i"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zjd/3dpart/openmvs1.1.1/libs/Common/Log.cpp > CMakeFiles/Common.dir/Log.cpp.i

libs/Common/CMakeFiles/Common.dir/Log.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Common.dir/Log.cpp.s"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zjd/3dpart/openmvs1.1.1/libs/Common/Log.cpp -o CMakeFiles/Common.dir/Log.cpp.s

libs/Common/CMakeFiles/Common.dir/SML.cpp.o: libs/Common/CMakeFiles/Common.dir/flags.make
libs/Common/CMakeFiles/Common.dir/SML.cpp.o: ../libs/Common/SML.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zjd/3dpart/openmvs1.1.1/openmvs_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object libs/Common/CMakeFiles/Common.dir/SML.cpp.o"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Common.dir/SML.cpp.o -c /home/zjd/3dpart/openmvs1.1.1/libs/Common/SML.cpp

libs/Common/CMakeFiles/Common.dir/SML.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Common.dir/SML.cpp.i"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zjd/3dpart/openmvs1.1.1/libs/Common/SML.cpp > CMakeFiles/Common.dir/SML.cpp.i

libs/Common/CMakeFiles/Common.dir/SML.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Common.dir/SML.cpp.s"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zjd/3dpart/openmvs1.1.1/libs/Common/SML.cpp -o CMakeFiles/Common.dir/SML.cpp.s

libs/Common/CMakeFiles/Common.dir/Timer.cpp.o: libs/Common/CMakeFiles/Common.dir/flags.make
libs/Common/CMakeFiles/Common.dir/Timer.cpp.o: ../libs/Common/Timer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zjd/3dpart/openmvs1.1.1/openmvs_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object libs/Common/CMakeFiles/Common.dir/Timer.cpp.o"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Common.dir/Timer.cpp.o -c /home/zjd/3dpart/openmvs1.1.1/libs/Common/Timer.cpp

libs/Common/CMakeFiles/Common.dir/Timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Common.dir/Timer.cpp.i"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zjd/3dpart/openmvs1.1.1/libs/Common/Timer.cpp > CMakeFiles/Common.dir/Timer.cpp.i

libs/Common/CMakeFiles/Common.dir/Timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Common.dir/Timer.cpp.s"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zjd/3dpart/openmvs1.1.1/libs/Common/Timer.cpp -o CMakeFiles/Common.dir/Timer.cpp.s

libs/Common/CMakeFiles/Common.dir/Types.cpp.o: libs/Common/CMakeFiles/Common.dir/flags.make
libs/Common/CMakeFiles/Common.dir/Types.cpp.o: ../libs/Common/Types.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zjd/3dpart/openmvs1.1.1/openmvs_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object libs/Common/CMakeFiles/Common.dir/Types.cpp.o"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Common.dir/Types.cpp.o -c /home/zjd/3dpart/openmvs1.1.1/libs/Common/Types.cpp

libs/Common/CMakeFiles/Common.dir/Types.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Common.dir/Types.cpp.i"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zjd/3dpart/openmvs1.1.1/libs/Common/Types.cpp > CMakeFiles/Common.dir/Types.cpp.i

libs/Common/CMakeFiles/Common.dir/Types.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Common.dir/Types.cpp.s"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zjd/3dpart/openmvs1.1.1/libs/Common/Types.cpp -o CMakeFiles/Common.dir/Types.cpp.s

libs/Common/CMakeFiles/Common.dir/Util.cpp.o: libs/Common/CMakeFiles/Common.dir/flags.make
libs/Common/CMakeFiles/Common.dir/Util.cpp.o: ../libs/Common/Util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zjd/3dpart/openmvs1.1.1/openmvs_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object libs/Common/CMakeFiles/Common.dir/Util.cpp.o"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Common.dir/Util.cpp.o -c /home/zjd/3dpart/openmvs1.1.1/libs/Common/Util.cpp

libs/Common/CMakeFiles/Common.dir/Util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Common.dir/Util.cpp.i"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zjd/3dpart/openmvs1.1.1/libs/Common/Util.cpp > CMakeFiles/Common.dir/Util.cpp.i

libs/Common/CMakeFiles/Common.dir/Util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Common.dir/Util.cpp.s"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zjd/3dpart/openmvs1.1.1/libs/Common/Util.cpp -o CMakeFiles/Common.dir/Util.cpp.s

# Object files for target Common
Common_OBJECTS = \
"CMakeFiles/Common.dir/Common.cpp.o" \
"CMakeFiles/Common.dir/CUDA.cpp.o" \
"CMakeFiles/Common.dir/ConfigTable.cpp.o" \
"CMakeFiles/Common.dir/EventQueue.cpp.o" \
"CMakeFiles/Common.dir/Log.cpp.o" \
"CMakeFiles/Common.dir/SML.cpp.o" \
"CMakeFiles/Common.dir/Timer.cpp.o" \
"CMakeFiles/Common.dir/Types.cpp.o" \
"CMakeFiles/Common.dir/Util.cpp.o"

# External object files for target Common
Common_EXTERNAL_OBJECTS =

lib/libCommon.a: libs/Common/CMakeFiles/Common.dir/Common.cpp.o
lib/libCommon.a: libs/Common/CMakeFiles/Common.dir/CUDA.cpp.o
lib/libCommon.a: libs/Common/CMakeFiles/Common.dir/ConfigTable.cpp.o
lib/libCommon.a: libs/Common/CMakeFiles/Common.dir/EventQueue.cpp.o
lib/libCommon.a: libs/Common/CMakeFiles/Common.dir/Log.cpp.o
lib/libCommon.a: libs/Common/CMakeFiles/Common.dir/SML.cpp.o
lib/libCommon.a: libs/Common/CMakeFiles/Common.dir/Timer.cpp.o
lib/libCommon.a: libs/Common/CMakeFiles/Common.dir/Types.cpp.o
lib/libCommon.a: libs/Common/CMakeFiles/Common.dir/Util.cpp.o
lib/libCommon.a: libs/Common/CMakeFiles/Common.dir/build.make
lib/libCommon.a: libs/Common/CMakeFiles/Common.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zjd/3dpart/openmvs1.1.1/openmvs_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX static library ../../lib/libCommon.a"
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && $(CMAKE_COMMAND) -P CMakeFiles/Common.dir/cmake_clean_target.cmake
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Common.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libs/Common/CMakeFiles/Common.dir/build: lib/libCommon.a

.PHONY : libs/Common/CMakeFiles/Common.dir/build

libs/Common/CMakeFiles/Common.dir/clean:
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common && $(CMAKE_COMMAND) -P CMakeFiles/Common.dir/cmake_clean.cmake
.PHONY : libs/Common/CMakeFiles/Common.dir/clean

libs/Common/CMakeFiles/Common.dir/depend:
	cd /home/zjd/3dpart/openmvs1.1.1/openmvs_build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zjd/3dpart/openmvs1.1.1 /home/zjd/3dpart/openmvs1.1.1/libs/Common /home/zjd/3dpart/openmvs1.1.1/openmvs_build /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common /home/zjd/3dpart/openmvs1.1.1/openmvs_build/libs/Common/CMakeFiles/Common.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libs/Common/CMakeFiles/Common.dir/depend

