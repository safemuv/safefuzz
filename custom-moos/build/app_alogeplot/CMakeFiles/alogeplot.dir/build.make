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
include app_alogeplot/CMakeFiles/alogeplot.dir/depend.make

# Include the progress variables for this target.
include app_alogeplot/CMakeFiles/alogeplot.dir/progress.make

# Include the compile flags for this target's objects.
include app_alogeplot/CMakeFiles/alogeplot.dir/flags.make

app_alogeplot/CMakeFiles/alogeplot.dir/main.cpp.o: app_alogeplot/CMakeFiles/alogeplot.dir/flags.make
app_alogeplot/CMakeFiles/alogeplot.dir/main.cpp.o: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogeplot/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object app_alogeplot/CMakeFiles/alogeplot.dir/main.cpp.o"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogeplot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/alogeplot.dir/main.cpp.o -c /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogeplot/main.cpp

app_alogeplot/CMakeFiles/alogeplot.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/alogeplot.dir/main.cpp.i"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogeplot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogeplot/main.cpp > CMakeFiles/alogeplot.dir/main.cpp.i

app_alogeplot/CMakeFiles/alogeplot.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/alogeplot.dir/main.cpp.s"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogeplot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogeplot/main.cpp -o CMakeFiles/alogeplot.dir/main.cpp.s

app_alogeplot/CMakeFiles/alogeplot.dir/EPlotEngine.cpp.o: app_alogeplot/CMakeFiles/alogeplot.dir/flags.make
app_alogeplot/CMakeFiles/alogeplot.dir/EPlotEngine.cpp.o: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogeplot/EPlotEngine.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object app_alogeplot/CMakeFiles/alogeplot.dir/EPlotEngine.cpp.o"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogeplot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/alogeplot.dir/EPlotEngine.cpp.o -c /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogeplot/EPlotEngine.cpp

app_alogeplot/CMakeFiles/alogeplot.dir/EPlotEngine.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/alogeplot.dir/EPlotEngine.cpp.i"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogeplot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogeplot/EPlotEngine.cpp > CMakeFiles/alogeplot.dir/EPlotEngine.cpp.i

app_alogeplot/CMakeFiles/alogeplot.dir/EPlotEngine.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/alogeplot.dir/EPlotEngine.cpp.s"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogeplot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogeplot/EPlotEngine.cpp -o CMakeFiles/alogeplot.dir/EPlotEngine.cpp.s

# Object files for target alogeplot
alogeplot_OBJECTS = \
"CMakeFiles/alogeplot.dir/main.cpp.o" \
"CMakeFiles/alogeplot.dir/EPlotEngine.cpp.o"

# External object files for target alogeplot
alogeplot_EXTERNAL_OBJECTS =

/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogeplot: app_alogeplot/CMakeFiles/alogeplot.dir/main.cpp.o
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogeplot: app_alogeplot/CMakeFiles/alogeplot.dir/EPlotEngine.cpp.o
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogeplot: app_alogeplot/CMakeFiles/alogeplot.dir/build.make
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogeplot: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/liblogutils.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogeplot: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/libencounters.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogeplot: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/libmbutil.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogeplot: app_alogeplot/CMakeFiles/alogeplot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogeplot"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogeplot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/alogeplot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
app_alogeplot/CMakeFiles/alogeplot.dir/build: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/alogeplot

.PHONY : app_alogeplot/CMakeFiles/alogeplot.dir/build

app_alogeplot/CMakeFiles/alogeplot.dir/clean:
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogeplot && $(CMAKE_COMMAND) -P CMakeFiles/alogeplot.dir/cmake_clean.cmake
.PHONY : app_alogeplot/CMakeFiles/alogeplot.dir/clean

app_alogeplot/CMakeFiles/alogeplot.dir/depend:
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/app_alogeplot /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogeplot /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/app_alogeplot/CMakeFiles/alogeplot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app_alogeplot/CMakeFiles/alogeplot.dir/depend

