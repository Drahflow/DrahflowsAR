package name.drahflow.ar;

import android.opengl.Matrix;
import android.util.Log;

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

		SVO_processFrame(lastFrame.getIntensities(), transformation);
		lastFrame.setTransformation(transformation);

		long end = System.nanoTime();
		Log.e("AR", "optimization took: " + (float)(end - start) / 1000000 + " ms");
  }

  static {
     System.loadLibrary("cameratracker");
  }
	public native static void SVO_prepare(int width, int height, float fx, float fy, double cx, double cy);
	public native static void SVO_processFrame(float[] intensities, float[] transformation);
}
