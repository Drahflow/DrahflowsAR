#include <jni.h>
#include "CameraTracker.h"

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <iostream>

static vk::AbstractCamera *camera;
static svo::FrameHandlerMono *frameHandler;

JNIEXPORT void JNICALL Java_name_drahflow_ar_CameraTracker_SVO_1prepare
  (JNIEnv *, jclass, jint, jint, jfloat, jfloat, jdouble, jdouble) {
  camera = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
  frameHandler = new svo::FrameHandlerMono(camera);
  frameHandler->start();
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_CameraTracker_SVO_1processFrame
  (JNIEnv *, jclass, jfloatArray, jfloatArray) {
}
