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


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/hojin/ADplatform/hmclmobil/src/acados

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hojin/ADplatform/hmclmobil/src/acados/build

# Include any dependencies generated for this target.
include external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/depend.make

# Include the progress variables for this target.
include external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/progress.make

# Include the compile flags for this target's objects.
include external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/flags.make

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/flags.make
external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o: ../external/blasfeo/examples/example_s_riccati_recursion.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hojin/ADplatform/hmclmobil/src/acados/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o"
	cd /home/hojin/ADplatform/hmclmobil/src/acados/build/external/blasfeo/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o   -c /home/hojin/ADplatform/hmclmobil/src/acados/external/blasfeo/examples/example_s_riccati_recursion.c

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.i"
	cd /home/hojin/ADplatform/hmclmobil/src/acados/build/external/blasfeo/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hojin/ADplatform/hmclmobil/src/acados/external/blasfeo/examples/example_s_riccati_recursion.c > CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.i

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.s"
	cd /home/hojin/ADplatform/hmclmobil/src/acados/build/external/blasfeo/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hojin/ADplatform/hmclmobil/src/acados/external/blasfeo/examples/example_s_riccati_recursion.c -o CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.s

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o.requires:

.PHONY : external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o.requires

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o.provides: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o.requires
	$(MAKE) -f external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/build.make external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o.provides.build
.PHONY : external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o.provides

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o.provides.build: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o


external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/flags.make
external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o: ../external/blasfeo/examples/tools.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hojin/ADplatform/hmclmobil/src/acados/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o"
	cd /home/hojin/ADplatform/hmclmobil/src/acados/build/external/blasfeo/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/example_s_riccati_recursion.dir/tools.c.o   -c /home/hojin/ADplatform/hmclmobil/src/acados/external/blasfeo/examples/tools.c

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/example_s_riccati_recursion.dir/tools.c.i"
	cd /home/hojin/ADplatform/hmclmobil/src/acados/build/external/blasfeo/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hojin/ADplatform/hmclmobil/src/acados/external/blasfeo/examples/tools.c > CMakeFiles/example_s_riccati_recursion.dir/tools.c.i

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/example_s_riccati_recursion.dir/tools.c.s"
	cd /home/hojin/ADplatform/hmclmobil/src/acados/build/external/blasfeo/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hojin/ADplatform/hmclmobil/src/acados/external/blasfeo/examples/tools.c -o CMakeFiles/example_s_riccati_recursion.dir/tools.c.s

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o.requires:

.PHONY : external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o.requires

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o.provides: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o.requires
	$(MAKE) -f external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/build.make external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o.provides.build
.PHONY : external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o.provides

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o.provides.build: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o


# Object files for target example_s_riccati_recursion
example_s_riccati_recursion_OBJECTS = \
"CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o" \
"CMakeFiles/example_s_riccati_recursion.dir/tools.c.o"

# External object files for target example_s_riccati_recursion
example_s_riccati_recursion_EXTERNAL_OBJECTS =

external/blasfeo/examples/example_s_riccati_recursion: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o
external/blasfeo/examples/example_s_riccati_recursion: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o
external/blasfeo/examples/example_s_riccati_recursion: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/build.make
external/blasfeo/examples/example_s_riccati_recursion: external/blasfeo/libblasfeo.so
external/blasfeo/examples/example_s_riccati_recursion: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hojin/ADplatform/hmclmobil/src/acados/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable example_s_riccati_recursion"
	cd /home/hojin/ADplatform/hmclmobil/src/acados/build/external/blasfeo/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_s_riccati_recursion.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/build: external/blasfeo/examples/example_s_riccati_recursion

.PHONY : external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/build

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/requires: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o.requires
external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/requires: external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o.requires

.PHONY : external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/requires

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/clean:
	cd /home/hojin/ADplatform/hmclmobil/src/acados/build/external/blasfeo/examples && $(CMAKE_COMMAND) -P CMakeFiles/example_s_riccati_recursion.dir/cmake_clean.cmake
.PHONY : external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/clean

external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/depend:
	cd /home/hojin/ADplatform/hmclmobil/src/acados/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hojin/ADplatform/hmclmobil/src/acados /home/hojin/ADplatform/hmclmobil/src/acados/external/blasfeo/examples /home/hojin/ADplatform/hmclmobil/src/acados/build /home/hojin/ADplatform/hmclmobil/src/acados/build/external/blasfeo/examples /home/hojin/ADplatform/hmclmobil/src/acados/build/external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : external/blasfeo/examples/CMakeFiles/example_s_riccati_recursion.dir/depend

