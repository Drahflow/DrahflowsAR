package name.drahflow.ar;

public class GlobalState {
	public DrahflowsAR main;
	public CameraTracker cameraTracker;
	public VideoHistory videoHistory;
	public SensorTracker sensorTracker;
	public GestureTracker gestureTracker;
	public name.drahflow.ar.geometry.View view;

	public long displayLag = 90000000;
	public float eyeShift = 0.08f;
	public float eyeDistance = 0.0246f;
	public float vrScale = 0.2f;
	public float cameraDistance = 0.02f;
	public float accelerometerScale = 4f;
	public float gestureSizeAtOne = 1f;
	public float[] gestureOffset = new float[] { 0f, 0f };

	public void onPause() {
		if(cameraTracker != null) cameraTracker.onPause();
		if(sensorTracker != null) sensorTracker.onPause();
	}

	public void onResume() {
		if(cameraTracker != null) cameraTracker.onResume();
		if(sensorTracker != null) sensorTracker.onResume();
	}
}
