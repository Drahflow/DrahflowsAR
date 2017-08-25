package name.drahflow.ar;

class GestureTracker {
	// 2d + homogene coordinates relative movement in camera image
	float[] gestureTransformation = new float[9];

	public GestureTracker() {
	}

	public boolean isTrackingEstablished() {
		JNI.Gesture_getTransformationRelative(System.nanoTime(), gestureTransformation);

		// non-tracking marked via all-zero
		return gestureTransformation[8] != 0;
	}
}
