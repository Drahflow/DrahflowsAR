package name.drahflow.ar;

public class GlobalState {
	CameraTracker cameraTracker;
	VideoHistory videoHistory;

	SensorTracker sensorTracker;

	GestureTracker gestureTracker;

	public void onPause() {
		if(cameraTracker != null) cameraTracker.onPause();
		if(sensorTracker != null) sensorTracker.onPause();
	}

	public void onResume() {
		if(cameraTracker != null) cameraTracker.onResume();
		if(sensorTracker != null) sensorTracker.onResume();
	}
}
