package name.drahflow.ar;

import android.opengl.Matrix;
import android.util.Log;
import android.hardware.SensorEvent;

public class CameraTracker {
  private int width;
  private int height;
	private VideoHistory history;

  public CameraTracker(int _width, int _height, final VideoHistory _history) {
    width = _width;
    height = _height;
		history = _history;

		// TODO: drop (or forward) unused calibration parameters
		SVO_prepare(width, height, 0.0f, 0.0f, 0.0, 0.0);
	}

  public void processFrame() {
		long start = System.nanoTime();
		float[] transformation = new float[7];
		VideoFrame lastFrame = history.getLastFrame();

		Log.e("AR", "Camera processing on thread: " + Thread.currentThread().getName());
		SVO_processFrame(lastFrame.getIntensities(), transformation, lastFrame.getTimestamp());
		lastFrame.setTransformation(transformation);

		long end = System.nanoTime();
		Log.e("AR", "optimization took: " + (float)(end - start) / 1000000 + " ms");
  }

	public void processAccelerometerEvent(SensorEvent e) {
		Log.e("AR", "Accelerometer processing on thread: " + Thread.currentThread().getName());
		SVO_processAccelerometer(e.values, e.timestamp);
	}

	public void processGyroscopeEvent(SensorEvent e) {
		Log.e("AR", "Gyroscope processing on thread: " + Thread.currentThread().getName());
		SVO_processGyroscope(e.values, e.timestamp);
	}

	public void getTransformationAt(long time_nano, float[] transformation) {
		Log.e("AR", "Pose estimation on thread: " + Thread.currentThread().getName());
		SVO_getTransformation(time_nano, transformation);

		Log.e("AR", "pose: " +
				String.format("%8.6f,%8.6f,%8.6f @ %8.6f,%8.6f,%8.6f,%8.6f",
					transformation[0], transformation[1], transformation[2],
					transformation[3], transformation[4], transformation[5], transformation[6]));
	}

  static {
     System.loadLibrary("cameratracker");
  }
	public native static void SVO_prepare(int width, int height, float fx, float fy, double cx, double cy);
	public native static void SVO_processFrame(float[] intensities, float[] transformation, long time_nano);
	public native static void SVO_processAccelerometer(float[] xyz, long time_nano);
	public native static void SVO_processGyroscope(float[] xyz, long time_nano);
	public native static void SVO_getTransformation(long time_nano, float[] transformation);
}
