package name.drahflow.ar;

public class GlobalState {
	DrahflowsAR main;
	CameraTracker cameraTracker;
	VideoHistory videoHistory;
	SensorTracker sensorTracker;
	GestureTracker gestureTracker;

	float cameraDistance = 0.03f;
	float eyeDistance = 0.07f;
	float eyeLookDistance = 0.07f;
	float mapScale = 2f;

	public void onPause() {
		if(cameraTracker != null) cameraTracker.onPause();
		if(sensorTracker != null) sensorTracker.onPause();
	}

	public void onResume() {
		if(cameraTracker != null) cameraTracker.onResume();
		if(sensorTracker != null) sensorTracker.onResume();
	}
}
