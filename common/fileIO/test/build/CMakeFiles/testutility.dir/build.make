# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test/build

# Include any dependencies generated for this target.
include CMakeFiles/testutility.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/testutility.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/testutility.dir/flags.make

CMakeFiles/testutility.dir/testutilityIO.cc.o: CMakeFiles/testutility.dir/flags.make
CMakeFiles/testutility.dir/testutilityIO.cc.o: ../testutilityIO.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/testutility.dir/testutilityIO.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testutility.dir/testutilityIO.cc.o -c /home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test/testutilityIO.cc

CMakeFiles/testutility.dir/testutilityIO.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testutility.dir/testutilityIO.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test/testutilityIO.cc > CMakeFiles/testutility.dir/testutilityIO.cc.i

CMakeFiles/testutility.dir/testutilityIO.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testutility.dir/testutilityIO.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test/testutilityIO.cc -o CMakeFiles/testutility.dir/testutilityIO.cc.s

CMakeFiles/testutility.dir/testutilityIO.cc.o.requires:

.PHONY : CMakeFiles/testutility.dir/testutilityIO.cc.o.requires

CMakeFiles/testutility.dir/testutilityIO.cc.o.provides: CMakeFiles/testutility.dir/testutilityIO.cc.o.requires
	$(MAKE) -f CMakeFiles/testutility.dir/build.make CMakeFiles/testutility.dir/testutilityIO.cc.o.provides.build
.PHONY : CMakeFiles/testutility.dir/testutilityIO.cc.o.provides

CMakeFiles/testutility.dir/testutilityIO.cc.o.provides.build: CMakeFiles/testutility.dir/testutilityIO.cc.o


# Object files for target testutility
testutility_OBJECTS = \
"CMakeFiles/testutility.dir/testutilityIO.cc.o"

# External object files for target testutility
testutility_EXTERNAL_OBJECTS =

testutility: CMakeFiles/testutility.dir/testutilityIO.cc.o
testutility: CMakeFiles/testutility.dir/build.make
testutility: CMakeFiles/testutility.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable testutility"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testutility.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/testutility.dir/build: testutility

.PHONY : CMakeFiles/testutility.dir/build

CMakeFiles/testutility.dir/requires: CMakeFiles/testutility.dir/testutilityIO.cc.o.requires

.PHONY : CMakeFiles/testutility.dir/requires

CMakeFiles/testutility.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/testutility.dir/cmake_clean.cmake
.PHONY : CMakeFiles/testutility.dir/clean

CMakeFiles/testutility.dir/depend:
	cd /home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test /home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test /home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test/build /home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test/build /home/skloe/Documents/毕业要紧/ASV-master20200410/ASV-master/common/fileIO/test/build/CMakeFiles/testutility.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/testutility.dir/depend
