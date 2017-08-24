package name.drahflow.ar;

class JNI {
  static {
     System.loadLibrary("ar_jni");
  }

	public native static void SVO_prepare(int width, int height, float fx, float fy, double cx, double cy);
	public native static void SVO_processFrame(float[] intensities, long time_nano);
	public native static void SVO_processAccelerometer(float[] xyz, long time_nano);
	public native static void SVO_processGyroscope(float[] xyz, long time_nano);
	public native static void SVO_getTransformation(long time_nano, float[] transformation);
	public native static boolean SVO_hasGoodTracking();
	public native static void SVO_setAccelerometerScale(float scale);

	public native static void Gesture_setMarker(int xs, int ys, int xe, int ye);
	public native static void Gesture_processFrame(float[] debugImage);
	public native static void Gesture_getTransformationRelative(long time_nano, float[] transformation);
}
