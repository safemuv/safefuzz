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
include app_aloggrep/CMakeFiles/aloggrep.dir/depend.make

# Include the progress variables for this target.
include app_aloggrep/CMakeFiles/aloggrep.dir/progress.make

# Include the compile flags for this target's objects.
include app_aloggrep/CMakeFiles/aloggrep.dir/flags.make

app_aloggrep/CMakeFiles/aloggrep.dir/main.cpp.o: app_aloggrep/CMakeFiles/aloggrep.dir/flags.make
app_aloggrep/CMakeFiles/aloggrep.dir/main.cpp.o: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_aloggrep/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object app_aloggrep/CMakeFiles/aloggrep.dir/main.cpp.o"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_aloggrep && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aloggrep.dir/main.cpp.o -c /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_aloggrep/main.cpp

app_aloggrep/CMakeFiles/aloggrep.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aloggrep.dir/main.cpp.i"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_aloggrep && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_aloggrep/main.cpp > CMakeFiles/aloggrep.dir/main.cpp.i

app_aloggrep/CMakeFiles/aloggrep.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aloggrep.dir/main.cpp.s"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_aloggrep && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_aloggrep/main.cpp -o CMakeFiles/aloggrep.dir/main.cpp.s

app_aloggrep/CMakeFiles/aloggrep.dir/GrepHandler.cpp.o: app_aloggrep/CMakeFiles/aloggrep.dir/flags.make
app_aloggrep/CMakeFiles/aloggrep.dir/GrepHandler.cpp.o: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_aloggrep/GrepHandler.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object app_aloggrep/CMakeFiles/aloggrep.dir/GrepHandler.cpp.o"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_aloggrep && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aloggrep.dir/GrepHandler.cpp.o -c /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_aloggrep/GrepHandler.cpp

app_aloggrep/CMakeFiles/aloggrep.dir/GrepHandler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aloggrep.dir/GrepHandler.cpp.i"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_aloggrep && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_aloggrep/GrepHandler.cpp > CMakeFiles/aloggrep.dir/GrepHandler.cpp.i

app_aloggrep/CMakeFiles/aloggrep.dir/GrepHandler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aloggrep.dir/GrepHandler.cpp.s"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_aloggrep && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_aloggrep/GrepHandler.cpp -o CMakeFiles/aloggrep.dir/GrepHandler.cpp.s

# Object files for target aloggrep
aloggrep_OBJECTS = \
"CMakeFiles/aloggrep.dir/main.cpp.o" \
"CMakeFiles/aloggrep.dir/GrepHandler.cpp.o"

# External object files for target aloggrep
aloggrep_EXTERNAL_OBJECTS =

/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/aloggrep: app_aloggrep/CMakeFiles/aloggrep.dir/main.cpp.o
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/aloggrep: app_aloggrep/CMakeFiles/aloggrep.dir/GrepHandler.cpp.o
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/aloggrep: app_aloggrep/CMakeFiles/aloggrep.dir/build.make
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/aloggrep: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/libmbutil.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/aloggrep: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/liblogutils.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/aloggrep: app_aloggrep/CMakeFiles/aloggrep.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/aloggrep"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_aloggrep && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aloggrep.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
app_aloggrep/CMakeFiles/aloggrep.dir/build: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/aloggrep

.PHONY : app_aloggrep/CMakeFiles/aloggrep.dir/build

app_aloggrep/CMakeFiles/aloggrep.dir/clean:
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_aloggrep && $(CMAKE_COMMAND) -P CMakeFiles/aloggrep.dir/cmake_clean.cmake
.PHONY : app_aloggrep/CMakeFiles/aloggrep.dir/clean

app_aloggrep/CMakeFiles/aloggrep.dir/depend:
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_aloggrep /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_aloggrep /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_aloggrep/CMakeFiles/aloggrep.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app_aloggrep/CMakeFiles/aloggrep.dir/depend

