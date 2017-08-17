################################################################################
# user build settings

SET(TRACE TRUE)
SET(HAVE_G2O FALSE)
SET(USE_ROS FALSE) # Set FALSE if you want to build this package without Catkin
SET(DEBUG_OUTPUT FALSE) # Only relevant if build without ROS

################################################################################

SET(PROJECT_NAME svo)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

SET(CMAKE_CXX_COMPILER "${PROJECT_SOURCE_DIR}/toolchain/bin/i686-linux-android-g++")

# Set definitions
IF(TRACE)
  ADD_DEFINITIONS(-DSVO_TRACE)
ENDIF()
IF(HAVE_G2O)
  ADD_DEFINITIONS(-DUSE_BUNDLE_ADJUSTMENT)
ENDIF()
IF(USE_ROS)
  ADD_DEFINITIONS(-DSVO_USE_ROS)
ENDIF()
IF(DEBUG_OUTPUT)
  ADD_DEFINITIONS(-DSVO_DEBUG_OUTPUT)
ENDIF()

# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
# Set build flags. Set IS_ARM on odroid board as environment variable
SET(CMAKE_CXX_FLAGS "-march=atom -m32 -ggdb -Wall -D_LINUX -D_REENTRANT -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -Wl,--hash-style=both")

# Add plain cmake packages
FIND_PACKAGE(Eigen REQUIRED)
# FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
)

# Set link libraries
LIST(APPEND LINK_LIBS
  # ${Boost_LIBRARIES}
  # jpeg png
  android log
  gnustl_shared
)

# Set sourcefiles
LIST(APPEND SOURCEFILES
  src/svo/frame_handler_mono.cpp
  src/svo/frame_handler_base.cpp
  src/svo/frame.cpp
  src/svo/point.cpp
  src/svo/map.cpp
  src/svo/pose_optimizer.cpp
  src/svo/initialization.cpp
  src/svo/matcher.cpp
  src/svo/reprojector.cpp
  src/svo/feature_alignment.cpp
  src/svo/feature_detection.cpp
  src/svo/depth_filter.cpp
  src/svo/config.cpp
  src/svo/sparse_img_align.cpp
  src/svo/faster_corner_10_sse.cpp
  src/svo/fast_10.cpp
  src/svo/fast_10_score.cpp
  src/svo/nonmax_3x3.cpp
  src/vikit/pinhole_camera.cpp
  src/vikit/robust_cost.cpp
  src/vikit/performance_monitor.cpp
  src/vikit/homography.cpp
  src/vikit/vision.cpp
  src/vikit/math_utils.cpp
  src/opencv/core/stl.cpp
  src/opencv/core/alloc.cpp
  src/opencv/core/matrix.cpp
  src/opencv/core/umatrix.cpp
  src/opencv/core/system.cpp
  src/opencv/core/copy.cpp
  src/opencv/core/array.cpp
  src/opencv/core/datastructs.cpp
  src/opencv/core/persistence.cpp
  src/opencv/core/convert.cpp
  src/opencv/core/matop.cpp
  src/opencv/core/stat.cpp
  src/opencv/core/arithm.cpp
  src/opencv/core/lapack.cpp
  src/opencv/core/matmul.cpp
  src/opencv/core/parallel.cpp
  src/opencv/core/hal_internal.cpp
  src/opencv/core/matrix_decomp.cpp
  src/opencv/core/merge.cpp
  src/opencv/core/kmeans.cpp
  src/opencv/core/split.cpp
  src/opencv/core/mathfuncs.cpp
  src/opencv/core/tables.cpp
  src/opencv/core/algorithm.cpp
  src/opencv/core/pca.cpp
  src/opencv/core/out.cpp
  src/opencv/core/rand.cpp
  src/opencv/core/dxt.cpp
  src/opencv/imgproc/undistort.cpp
  src/opencv/imgproc/imgwarp.cpp
  src/opencv/imgproc/pyramids.cpp
  src/opencv/imgproc/shapedescr.cpp
  src/opencv/imgproc/utils.cpp
  src/opencv/imgproc/color.cpp
  src/opencv/imgproc/demosaicing.cpp
  src/opencv/imgproc/sumpixels.cpp
  src/opencv/imgproc/histogram.cpp
  src/opencv/imgproc/tables.cpp
  src/opencv/imgproc/thresh.cpp
  src/opencv/imgproc/smooth.cpp
  src/opencv/imgproc/filter.cpp
  src/opencv/imgproc/templmatch.cpp
  src/opencv/calib3d/fundam.cpp
  src/opencv/calib3d/rho.cpp
  src/opencv/calib3d/ptsetreg.cpp
  src/opencv/calib3d/levmarq.cpp
  src/opencv/video/lkpyramid.cpp
  src/opencv/imgcodecs/loadsave.cpp
  src/opencv/imgcodecs/exif.cpp
  src/opencv/imgcodecs/grfmt_base.cpp
  # src/opencv/imgcodecs/grfmt_jpeg.cpp
  # src/opencv/imgcodecs/grfmt_jpeg2000.cpp
  # src/opencv/imgcodecs/grfmt_png.cpp
  src/opencv/features2d/feature2d.cpp
  src/opencv/features2d/brisk.cpp
  src/opencv/features2d/agast_score.cpp
  src/opencv/features2d/agast.cpp
  src/opencv/features2d/keypoint.cpp
  src/opencv/features2d/matchers.cpp
  src/opencv/xfeatures2d/freak.cpp
  src/boost/system/error_code.cpp
  src/boost/thread/pthread/thread.cpp
  src/boost/thread/pthread/once_atomic.cpp
  )

# Add g2o if available
IF(HAVE_G2O)
  FIND_PACKAGE(G2O REQUIRED)
  INCLUDE_DIRECTORIES(
    ${G2O_INCLUDE_DIR}
    /usr/include/suitesparse # for cholmod
  )
  LIST(APPEND LINK_LIBS
    ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG} ${G2O_TYPES_SBA} cholmod cxsparse
  )
  LIST(APPEND SOURCEFILES src/bundle_adjustment.cpp)
ENDIF(HAVE_G2O)

# Create svo library
ADD_LIBRARY(svo SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(svo ${LINK_LIBS})

################################################################################
# TESTS

# ADD_EXECUTABLE(test_feature_align test/test_feature_alignment.cpp)
# TARGET_LINK_LIBRARIES(test_feature_align svo)

ADD_EXECUTABLE(test_pipeline test/test_pipeline.cpp)
TARGET_LINK_LIBRARIES(test_pipeline svo)

# ADD_EXECUTABLE(test_matcher test/test_matcher.cpp)
# TARGET_LINK_LIBRARIES(test_matcher svo)
#
# ADD_EXECUTABLE(test_feature_detection test/test_feature_detection.cpp)
# TARGET_LINK_LIBRARIES(test_feature_detection svo)
#
# ADD_EXECUTABLE(test_depth_filter test/test_depth_filter.cpp)
# TARGET_LINK_LIBRARIES(test_depth_filter svo)
#
# ADD_EXECUTABLE(test_sparse_img_align test/test_sparse_img_align.cpp)
# TARGET_LINK_LIBRARIES(test_sparse_img_align svo)
#
# ADD_EXECUTABLE(test_pose_optimizer test/test_pose_optimizer.cpp)
# TARGET_LINK_LIBRARIES(test_pose_optimizer svo)
