# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/GraphCut-Textures.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/GraphCut-Textures.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/GraphCut-Textures.dir/flags.make

CMakeFiles/GraphCut-Textures.dir/main.cpp.o: CMakeFiles/GraphCut-Textures.dir/flags.make
CMakeFiles/GraphCut-Textures.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/GraphCut-Textures.dir/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/GraphCut-Textures.dir/main.cpp.o -c /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/main.cpp

CMakeFiles/GraphCut-Textures.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GraphCut-Textures.dir/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/main.cpp > CMakeFiles/GraphCut-Textures.dir/main.cpp.i

CMakeFiles/GraphCut-Textures.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GraphCut-Textures.dir/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/main.cpp -o CMakeFiles/GraphCut-Textures.dir/main.cpp.s

CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.o: CMakeFiles/GraphCut-Textures.dir/flags.make
CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.o: ../src/texture.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.o -c /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/src/texture.cpp

CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/src/texture.cpp > CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.i

CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/src/texture.cpp -o CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.s

CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.o: CMakeFiles/GraphCut-Textures.dir/flags.make
CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.o: ../src/graphcut.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.o -c /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/src/graphcut.cpp

CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/src/graphcut.cpp > CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.i

CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/src/graphcut.cpp -o CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.s

CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.o: CMakeFiles/GraphCut-Textures.dir/flags.make
CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.o: ../src/MaxFlow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.o -c /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/src/MaxFlow.cpp

CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/src/MaxFlow.cpp > CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.i

CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/src/MaxFlow.cpp -o CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.s

# Object files for target GraphCut-Textures
GraphCut__Textures_OBJECTS = \
"CMakeFiles/GraphCut-Textures.dir/main.cpp.o" \
"CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.o" \
"CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.o" \
"CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.o"

# External object files for target GraphCut-Textures
GraphCut__Textures_EXTERNAL_OBJECTS =

GraphCut-Textures: CMakeFiles/GraphCut-Textures.dir/main.cpp.o
GraphCut-Textures: CMakeFiles/GraphCut-Textures.dir/src/texture.cpp.o
GraphCut-Textures: CMakeFiles/GraphCut-Textures.dir/src/graphcut.cpp.o
GraphCut-Textures: CMakeFiles/GraphCut-Textures.dir/src/MaxFlow.cpp.o
GraphCut-Textures: CMakeFiles/GraphCut-Textures.dir/build.make
GraphCut-Textures: /usr/local/lib/libopencv_gapi.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_stitching.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_alphamat.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_aruco.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_bgsegm.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_bioinspired.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_ccalib.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_dnn_objdetect.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_dnn_superres.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_dpm.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_face.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_freetype.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_fuzzy.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_hfs.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_img_hash.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_intensity_transform.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_line_descriptor.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_mcc.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_quality.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_rapid.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_reg.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_rgbd.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_saliency.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_sfm.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_stereo.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_structured_light.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_superres.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_surface_matching.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_tracking.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_videostab.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_viz.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_xfeatures2d.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_xobjdetect.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_xphoto.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_highgui.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_shape.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_datasets.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_plot.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_text.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_dnn.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_ml.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_phase_unwrapping.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_optflow.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_ximgproc.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_video.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_videoio.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_imgcodecs.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_objdetect.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_calib3d.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_features2d.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_flann.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_photo.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_imgproc.4.5.0.dylib
GraphCut-Textures: /usr/local/lib/libopencv_core.4.5.0.dylib
GraphCut-Textures: CMakeFiles/GraphCut-Textures.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable GraphCut-Textures"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/GraphCut-Textures.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/GraphCut-Textures.dir/build: GraphCut-Textures

.PHONY : CMakeFiles/GraphCut-Textures.dir/build

CMakeFiles/GraphCut-Textures.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/GraphCut-Textures.dir/cmake_clean.cmake
.PHONY : CMakeFiles/GraphCut-Textures.dir/clean

CMakeFiles/GraphCut-Textures.dir/depend:
	cd /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/cmake-build-debug /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/cmake-build-debug /Users/andy/Desktop/大四上/媒体计算/project1/GraphCut-Textures/cmake-build-debug/CMakeFiles/GraphCut-Textures.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/GraphCut-Textures.dir/depend

