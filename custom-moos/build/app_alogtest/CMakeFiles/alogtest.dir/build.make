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
CMAKE_SOURCE_DIR = /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build

# Include any dependencies generated for this target.
include app_alogtest/CMakeFiles/alogtest.dir/depend.make

# Include the progress variables for this target.
include app_alogtest/CMakeFiles/alogtest.dir/progress.make

# Include the compile flags for this target's objects.
include app_alogtest/CMakeFiles/alogtest.dir/flags.make

app_alogtest/CMakeFiles/alogtest.dir/main.cpp.o: app_alogtest/CMakeFiles/alogtest.dir/flags.make
app_alogtest/CMakeFiles/alogtest.dir/main.cpp.o: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object app_alogtest/CMakeFiles/alogtest.dir/main.cpp.o"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/alogtest.dir/main.cpp.o -c /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/main.cpp

app_alogtest/CMakeFiles/alogtest.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/alogtest.dir/main.cpp.i"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/main.cpp > CMakeFiles/alogtest.dir/main.cpp.i

app_alogtest/CMakeFiles/alogtest.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/alogtest.dir/main.cpp.s"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/main.cpp -o CMakeFiles/alogtest.dir/main.cpp.s

app_alogtest/CMakeFiles/alogtest.dir/LogTest.cpp.o: app_alogtest/CMakeFiles/alogtest.dir/flags.make
app_alogtest/CMakeFiles/alogtest.dir/LogTest.cpp.o: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/LogTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object app_alogtest/CMakeFiles/alogtest.dir/LogTest.cpp.o"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/alogtest.dir/LogTest.cpp.o -c /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/LogTest.cpp

app_alogtest/CMakeFiles/alogtest.dir/LogTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/alogtest.dir/LogTest.cpp.i"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/LogTest.cpp > CMakeFiles/alogtest.dir/LogTest.cpp.i

app_alogtest/CMakeFiles/alogtest.dir/LogTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/alogtest.dir/LogTest.cpp.s"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/LogTest.cpp -o CMakeFiles/alogtest.dir/LogTest.cpp.s

app_alogtest/CMakeFiles/alogtest.dir/LogTester.cpp.o: app_alogtest/CMakeFiles/alogtest.dir/flags.make
app_alogtest/CMakeFiles/alogtest.dir/LogTester.cpp.o: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/LogTester.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object app_alogtest/CMakeFiles/alogtest.dir/LogTester.cpp.o"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/alogtest.dir/LogTester.cpp.o -c /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/LogTester.cpp

app_alogtest/CMakeFiles/alogtest.dir/LogTester.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/alogtest.dir/LogTester.cpp.i"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/LogTester.cpp > CMakeFiles/alogtest.dir/LogTester.cpp.i

app_alogtest/CMakeFiles/alogtest.dir/LogTester.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/alogtest.dir/LogTester.cpp.s"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest/LogTester.cpp -o CMakeFiles/alogtest.dir/LogTester.cpp.s

# Object files for target alogtest
alogtest_OBJECTS = \
"CMakeFiles/alogtest.dir/main.cpp.o" \
"CMakeFiles/alogtest.dir/LogTest.cpp.o" \
"CMakeFiles/alogtest.dir/LogTester.cpp.o"

# External object files for target alogtest
alogtest_EXTERNAL_OBJECTS =

/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogtest: app_alogtest/CMakeFiles/alogtest.dir/main.cpp.o
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogtest: app_alogtest/CMakeFiles/alogtest.dir/LogTest.cpp.o
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogtest: app_alogtest/CMakeFiles/alogtest.dir/LogTester.cpp.o
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogtest: app_alogtest/CMakeFiles/alogtest.dir/build.make
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogtest: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/libmbutil.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogtest: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/liblogic.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogtest: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/liblogutils.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogtest: app_alogtest/CMakeFiles/alogtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogtest"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/alogtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
app_alogtest/CMakeFiles/alogtest.dir/build: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogtest

.PHONY : app_alogtest/CMakeFiles/alogtest.dir/build

app_alogtest/CMakeFiles/alogtest.dir/clean:
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest && $(CMAKE_COMMAND) -P CMakeFiles/alogtest.dir/cmake_clean.cmake
.PHONY : app_alogtest/CMakeFiles/alogtest.dir/clean

app_alogtest/CMakeFiles/alogtest.dir/depend:
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogtest /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogtest/CMakeFiles/alogtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app_alogtest/CMakeFiles/alogtest.dir/depend

