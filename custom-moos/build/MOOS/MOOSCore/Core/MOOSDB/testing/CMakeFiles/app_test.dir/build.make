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
CMAKE_SOURCE_DIR = /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore

# Include any dependencies generated for this target.
include Core/MOOSDB/testing/CMakeFiles/app_test.dir/depend.make

# Include the progress variables for this target.
include Core/MOOSDB/testing/CMakeFiles/app_test.dir/progress.make

# Include the compile flags for this target's objects.
include Core/MOOSDB/testing/CMakeFiles/app_test.dir/flags.make

Core/MOOSDB/testing/CMakeFiles/app_test.dir/DBAppTest.cpp.o: Core/MOOSDB/testing/CMakeFiles/app_test.dir/flags.make
Core/MOOSDB/testing/CMakeFiles/app_test.dir/DBAppTest.cpp.o: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/MOOSDB/testing/DBAppTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Core/MOOSDB/testing/CMakeFiles/app_test.dir/DBAppTest.cpp.o"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore/Core/MOOSDB/testing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/app_test.dir/DBAppTest.cpp.o -c /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/MOOSDB/testing/DBAppTest.cpp

Core/MOOSDB/testing/CMakeFiles/app_test.dir/DBAppTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app_test.dir/DBAppTest.cpp.i"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore/Core/MOOSDB/testing && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/MOOSDB/testing/DBAppTest.cpp > CMakeFiles/app_test.dir/DBAppTest.cpp.i

Core/MOOSDB/testing/CMakeFiles/app_test.dir/DBAppTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app_test.dir/DBAppTest.cpp.s"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore/Core/MOOSDB/testing && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/MOOSDB/testing/DBAppTest.cpp -o CMakeFiles/app_test.dir/DBAppTest.cpp.s

# Object files for target app_test
app_test_OBJECTS = \
"CMakeFiles/app_test.dir/DBAppTest.cpp.o"

# External object files for target app_test
app_test_EXTERNAL_OBJECTS =

/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/tests/app_test: Core/MOOSDB/testing/CMakeFiles/app_test.dir/DBAppTest.cpp.o
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/tests/app_test: Core/MOOSDB/testing/CMakeFiles/app_test.dir/build.make
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/tests/app_test: lib/libMOOS.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/tests/app_test: Core/MOOSDB/testing/CMakeFiles/app_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/tests/app_test"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore/Core/MOOSDB/testing && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/app_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Core/MOOSDB/testing/CMakeFiles/app_test.dir/build: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/tests/app_test

.PHONY : Core/MOOSDB/testing/CMakeFiles/app_test.dir/build

Core/MOOSDB/testing/CMakeFiles/app_test.dir/clean:
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore/Core/MOOSDB/testing && $(CMAKE_COMMAND) -P CMakeFiles/app_test.dir/cmake_clean.cmake
.PHONY : Core/MOOSDB/testing/CMakeFiles/app_test.dir/clean

Core/MOOSDB/testing/CMakeFiles/app_test.dir/depend:
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/MOOSDB/testing /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore/Core/MOOSDB/testing /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore/Core/MOOSDB/testing/CMakeFiles/app_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Core/MOOSDB/testing/CMakeFiles/app_test.dir/depend

