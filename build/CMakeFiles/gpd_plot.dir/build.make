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
CMAKE_SOURCE_DIR = /home/tasbolat/some_python_examples/GRASP/gpd

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tasbolat/some_python_examples/GRASP/gpd/build

# Include any dependencies generated for this target.
include CMakeFiles/gpd_plot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gpd_plot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gpd_plot.dir/flags.make

CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.o: CMakeFiles/gpd_plot.dir/flags.make
CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.o: ../src/gpd/util/plot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tasbolat/some_python_examples/GRASP/gpd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.o -c /home/tasbolat/some_python_examples/GRASP/gpd/src/gpd/util/plot.cpp

CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tasbolat/some_python_examples/GRASP/gpd/src/gpd/util/plot.cpp > CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.i

CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tasbolat/some_python_examples/GRASP/gpd/src/gpd/util/plot.cpp -o CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.s

# Object files for target gpd_plot
gpd_plot_OBJECTS = \
"CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.o"

# External object files for target gpd_plot
gpd_plot_EXTERNAL_OBJECTS =

libgpd_plot.a: CMakeFiles/gpd_plot.dir/src/gpd/util/plot.cpp.o
libgpd_plot.a: CMakeFiles/gpd_plot.dir/build.make
libgpd_plot.a: CMakeFiles/gpd_plot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tasbolat/some_python_examples/GRASP/gpd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libgpd_plot.a"
	$(CMAKE_COMMAND) -P CMakeFiles/gpd_plot.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gpd_plot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gpd_plot.dir/build: libgpd_plot.a

.PHONY : CMakeFiles/gpd_plot.dir/build

CMakeFiles/gpd_plot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gpd_plot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gpd_plot.dir/clean

CMakeFiles/gpd_plot.dir/depend:
	cd /home/tasbolat/some_python_examples/GRASP/gpd/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tasbolat/some_python_examples/GRASP/gpd /home/tasbolat/some_python_examples/GRASP/gpd /home/tasbolat/some_python_examples/GRASP/gpd/build /home/tasbolat/some_python_examples/GRASP/gpd/build /home/tasbolat/some_python_examples/GRASP/gpd/build/CMakeFiles/gpd_plot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gpd_plot.dir/depend

