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
include uCommand/CMakeFiles/uCommand.dir/depend.make

# Include the progress variables for this target.
include uCommand/CMakeFiles/uCommand.dir/progress.make

# Include the compile flags for this target's objects.
include uCommand/CMakeFiles/uCommand.dir/flags.make

uCommand/CMakeFiles/uCommand.dir/UCMD_Info.cpp.o: uCommand/CMakeFiles/uCommand.dir/flags.make
uCommand/CMakeFiles/uCommand.dir/UCMD_Info.cpp.o: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/UCMD_Info.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object uCommand/CMakeFiles/uCommand.dir/UCMD_Info.cpp.o"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/uCommand.dir/UCMD_Info.cpp.o -c /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/UCMD_Info.cpp

uCommand/CMakeFiles/uCommand.dir/UCMD_Info.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/uCommand.dir/UCMD_Info.cpp.i"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/UCMD_Info.cpp > CMakeFiles/uCommand.dir/UCMD_Info.cpp.i

uCommand/CMakeFiles/uCommand.dir/UCMD_Info.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/uCommand.dir/UCMD_Info.cpp.s"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/UCMD_Info.cpp -o CMakeFiles/uCommand.dir/UCMD_Info.cpp.s

uCommand/CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.o: uCommand/CMakeFiles/uCommand.dir/flags.make
uCommand/CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.o: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/UCMD_MOOSApp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object uCommand/CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.o"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.o -c /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/UCMD_MOOSApp.cpp

uCommand/CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.i"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/UCMD_MOOSApp.cpp > CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.i

uCommand/CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.s"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/UCMD_MOOSApp.cpp -o CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.s

uCommand/CMakeFiles/uCommand.dir/main.cpp.o: uCommand/CMakeFiles/uCommand.dir/flags.make
uCommand/CMakeFiles/uCommand.dir/main.cpp.o: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object uCommand/CMakeFiles/uCommand.dir/main.cpp.o"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/uCommand.dir/main.cpp.o -c /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/main.cpp

uCommand/CMakeFiles/uCommand.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/uCommand.dir/main.cpp.i"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/main.cpp > CMakeFiles/uCommand.dir/main.cpp.i

uCommand/CMakeFiles/uCommand.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/uCommand.dir/main.cpp.s"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand/main.cpp -o CMakeFiles/uCommand.dir/main.cpp.s

# Object files for target uCommand
uCommand_OBJECTS = \
"CMakeFiles/uCommand.dir/UCMD_Info.cpp.o" \
"CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.o" \
"CMakeFiles/uCommand.dir/main.cpp.o"

# External object files for target uCommand
uCommand_EXTERNAL_OBJECTS =

/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand: uCommand/CMakeFiles/uCommand.dir/UCMD_Info.cpp.o
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand: uCommand/CMakeFiles/uCommand.dir/UCMD_MOOSApp.cpp.o
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand: uCommand/CMakeFiles/uCommand.dir/main.cpp.o
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand: uCommand/CMakeFiles/uCommand.dir/build.make
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand: /home/jharbin/source/undersea/moos-ivp/MOOS/MOOSCore/lib/libMOOS.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/libucommand.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/libgenutil.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/libapputil.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/lib/libmbutil.a
/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand: uCommand/CMakeFiles/uCommand.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand"
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/uCommand.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
uCommand/CMakeFiles/uCommand.dir/build: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/bin/uCommand

.PHONY : uCommand/CMakeFiles/uCommand.dir/build

uCommand/CMakeFiles/uCommand.dir/clean:
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand && $(CMAKE_COMMAND) -P CMakeFiles/uCommand.dir/cmake_clean.cmake
.PHONY : uCommand/CMakeFiles/uCommand.dir/clean

uCommand/CMakeFiles/uCommand.dir/depend:
	cd /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/ivp/src/uCommand /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/uCommand/CMakeFiles/uCommand.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : uCommand/CMakeFiles/uCommand.dir/depend

