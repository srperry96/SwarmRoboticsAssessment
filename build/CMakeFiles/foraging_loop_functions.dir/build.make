# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build

# Include any dependencies generated for this target.
include CMakeFiles/foraging_loop_functions.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/foraging_loop_functions.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/foraging_loop_functions.dir/flags.make

CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o: CMakeFiles/foraging_loop_functions.dir/flags.make
CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o: /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/src/foraging_loop_functions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o -c /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/src/foraging_loop_functions.cpp

CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/src/foraging_loop_functions.cpp > CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.i

CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/src/foraging_loop_functions.cpp -o CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.s

CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.requires:

.PHONY : CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.requires

CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.provides: CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.requires
	$(MAKE) -f CMakeFiles/foraging_loop_functions.dir/build.make CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.provides.build
.PHONY : CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.provides

CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.provides.build: CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o


CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o: CMakeFiles/foraging_loop_functions.dir/flags.make
CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o: /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/src/foraging_qt_user_functions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o -c /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/src/foraging_qt_user_functions.cpp

CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/src/foraging_qt_user_functions.cpp > CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.i

CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/src/foraging_qt_user_functions.cpp -o CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.s

CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.requires:

.PHONY : CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.requires

CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.provides: CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.requires
	$(MAKE) -f CMakeFiles/foraging_loop_functions.dir/build.make CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.provides.build
.PHONY : CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.provides

CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.provides.build: CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o


CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o: CMakeFiles/foraging_loop_functions.dir/flags.make
CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o: foraging_loop_functions_automoc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o -c /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build/foraging_loop_functions_automoc.cpp

CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build/foraging_loop_functions_automoc.cpp > CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.i

CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build/foraging_loop_functions_automoc.cpp -o CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.s

CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.requires:

.PHONY : CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.requires

CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.provides: CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.requires
	$(MAKE) -f CMakeFiles/foraging_loop_functions.dir/build.make CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.provides.build
.PHONY : CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.provides

CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.provides.build: CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o


# Object files for target foraging_loop_functions
foraging_loop_functions_OBJECTS = \
"CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o" \
"CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o" \
"CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o"

# External object files for target foraging_loop_functions
foraging_loop_functions_EXTERNAL_OBJECTS =

libforaging_loop_functions.so: CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o
libforaging_loop_functions.so: CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o
libforaging_loop_functions.so: CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o
libforaging_loop_functions.so: CMakeFiles/foraging_loop_functions.dir/build.make
libforaging_loop_functions.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.5.1
libforaging_loop_functions.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.5.1
libforaging_loop_functions.so: /usr/lib/x86_64-linux-gnu/libglut.so
libforaging_loop_functions.so: /usr/lib/x86_64-linux-gnu/libXmu.so
libforaging_loop_functions.so: /usr/lib/x86_64-linux-gnu/libXi.so
libforaging_loop_functions.so: /usr/lib/x86_64-linux-gnu/libGLU.so
libforaging_loop_functions.so: /usr/lib/x86_64-linux-gnu/libGL.so
libforaging_loop_functions.so: /usr/lib/x86_64-linux-gnu/liblua5.2.so
libforaging_loop_functions.so: /usr/lib/x86_64-linux-gnu/libm.so
libforaging_loop_functions.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.5.1
libforaging_loop_functions.so: CMakeFiles/foraging_loop_functions.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared module libforaging_loop_functions.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/foraging_loop_functions.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/foraging_loop_functions.dir/build: libforaging_loop_functions.so

.PHONY : CMakeFiles/foraging_loop_functions.dir/build

CMakeFiles/foraging_loop_functions.dir/requires: CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.requires
CMakeFiles/foraging_loop_functions.dir/requires: CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.requires
CMakeFiles/foraging_loop_functions.dir/requires: CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.requires

.PHONY : CMakeFiles/foraging_loop_functions.dir/requires

CMakeFiles/foraging_loop_functions.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/foraging_loop_functions.dir/cmake_clean.cmake
.PHONY : CMakeFiles/foraging_loop_functions.dir/clean

CMakeFiles/foraging_loop_functions.dir/depend:
	cd /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/src /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/src /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build /home/sam/Documents/MScIR/SwarmIntelligence/SwarmRoboticsAssessment/build/CMakeFiles/foraging_loop_functions.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/foraging_loop_functions.dir/depend

