package name.drahflow.ar;

public class GlobalState {
	DrahflowsAR main;
	CameraTracker cameraTracker;
	VideoHistory videoHistory;
	SensorTracker sensorTracker;
	GestureTracker gestureTracker;

	float eyeShift = 0.08f;
	float eyeDistance = 0.0246f;
	float vrScale = 0.2f;
	float cameraDistance = 0.03f;
	float accelerometerScale = 4f;

	public void onPause() {
		if(cameraTracker != null) cameraTracker.onPause();
		if(sensorTracker != null) sensorTracker.onPause();
	}

	public void onResume() {
		if(cameraTracker != null) cameraTracker.onResume();
		if(sensorTracker != null) sensorTracker.onResume();
	}
}
