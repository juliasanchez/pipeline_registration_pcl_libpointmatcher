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
CMAKE_SOURCE_DIR = /home/julia/Desktop/pipeline_pcl_libpointmatcher/icp_registration

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/julia/Desktop/pipeline_pcl_libpointmatcher/build-icp_registration

# Include any dependencies generated for this target.
include CMakeFiles/icp_registration.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/icp_registration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/icp_registration.dir/flags.make

CMakeFiles/icp_registration.dir/icp_registration.cpp.o: CMakeFiles/icp_registration.dir/flags.make
CMakeFiles/icp_registration.dir/icp_registration.cpp.o: /home/julia/Desktop/pipeline_pcl_libpointmatcher/icp_registration/icp_registration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/julia/Desktop/pipeline_pcl_libpointmatcher/build-icp_registration/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/icp_registration.dir/icp_registration.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/icp_registration.dir/icp_registration.cpp.o -c /home/julia/Desktop/pipeline_pcl_libpointmatcher/icp_registration/icp_registration.cpp

CMakeFiles/icp_registration.dir/icp_registration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icp_registration.dir/icp_registration.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/julia/Desktop/pipeline_pcl_libpointmatcher/icp_registration/icp_registration.cpp > CMakeFiles/icp_registration.dir/icp_registration.cpp.i

CMakeFiles/icp_registration.dir/icp_registration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icp_registration.dir/icp_registration.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/julia/Desktop/pipeline_pcl_libpointmatcher/icp_registration/icp_registration.cpp -o CMakeFiles/icp_registration.dir/icp_registration.cpp.s

CMakeFiles/icp_registration.dir/icp_registration.cpp.o.requires:

.PHONY : CMakeFiles/icp_registration.dir/icp_registration.cpp.o.requires

CMakeFiles/icp_registration.dir/icp_registration.cpp.o.provides: CMakeFiles/icp_registration.dir/icp_registration.cpp.o.requires
	$(MAKE) -f CMakeFiles/icp_registration.dir/build.make CMakeFiles/icp_registration.dir/icp_registration.cpp.o.provides.build
.PHONY : CMakeFiles/icp_registration.dir/icp_registration.cpp.o.provides

CMakeFiles/icp_registration.dir/icp_registration.cpp.o.provides.build: CMakeFiles/icp_registration.dir/icp_registration.cpp.o


# Object files for target icp_registration
icp_registration_OBJECTS = \
"CMakeFiles/icp_registration.dir/icp_registration.cpp.o"

# External object files for target icp_registration
icp_registration_EXTERNAL_OBJECTS =

icp_registration: CMakeFiles/icp_registration.dir/icp_registration.cpp.o
icp_registration: CMakeFiles/icp_registration.dir/build.make
icp_registration: /usr/local/lib/libpointmatcher.so
icp_registration: /usr/lib/x86_64-linux-gnu/libboost_thread.so
icp_registration: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
icp_registration: /usr/lib/x86_64-linux-gnu/libboost_system.so
icp_registration: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
icp_registration: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
icp_registration: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
icp_registration: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
icp_registration: /usr/lib/x86_64-linux-gnu/libpthread.so
icp_registration: /usr/local/lib/libnabo.a
icp_registration: CMakeFiles/icp_registration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/julia/Desktop/pipeline_pcl_libpointmatcher/build-icp_registration/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable icp_registration"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/icp_registration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/icp_registration.dir/build: icp_registration

.PHONY : CMakeFiles/icp_registration.dir/build

CMakeFiles/icp_registration.dir/requires: CMakeFiles/icp_registration.dir/icp_registration.cpp.o.requires

.PHONY : CMakeFiles/icp_registration.dir/requires

CMakeFiles/icp_registration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/icp_registration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/icp_registration.dir/clean

CMakeFiles/icp_registration.dir/depend:
	cd /home/julia/Desktop/pipeline_pcl_libpointmatcher/build-icp_registration && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/julia/Desktop/pipeline_pcl_libpointmatcher/icp_registration /home/julia/Desktop/pipeline_pcl_libpointmatcher/icp_registration /home/julia/Desktop/pipeline_pcl_libpointmatcher/build-icp_registration /home/julia/Desktop/pipeline_pcl_libpointmatcher/build-icp_registration /home/julia/Desktop/pipeline_pcl_libpointmatcher/build-icp_registration/CMakeFiles/icp_registration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/icp_registration.dir/depend
