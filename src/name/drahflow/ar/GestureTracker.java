package name.drahflow.ar;

import android.opengl.Matrix;
import android.util.Log;

class GestureTracker {
	private GlobalState global;

	// 2d + homogene coordinates relative movement in camera image
	private float[] gestureTransformation = new float[9];

	// tracking pose matrix
	private float[] poseMatrix = new float[16];
	private float[] inversePoseMatrix = new float[16];

	public GestureTracker(GlobalState global) {
		this.global = global;
	}

	public boolean getPosition(float[] xyz) {
		JNI.Gesture_getTransformationRelative(System.nanoTime(), gestureTransformation);
		final float[] h = gestureTransformation;

		if(gestureTransformation[8] == 0) return false;

		float x = global.videoHistory.width * 0.50f;
		float y1 = global.videoHistory.height * 0.50f - 5.0f;
		float y2 = global.videoHistory.height * 0.50f + 5.0f;

		// apply transformation to given reference coordinates
		float tx = x * h[0] + y1 * h[1] + h[2];
		float ty = x * h[3] + y1 * h[4] + h[5];
		float tw = x * h[6] + y1 * h[7] + h[8];

		float bx = x * h[0] + y2 * h[1] + h[2];
		float by = x * h[3] + y2 * h[4] + h[5];
		float bw = x * h[6] + y2 * h[7] + h[8];

		tx /= tw;
		ty /= tw;
		bx /= bw;
		by /= bw;

		float size = (float)Math.sqrt(
			(tx - bx) * (tx - bx) +
			(ty - by) * (ty - by)
		);

		x = (tx + bx) / 2;
		float y = (ty + by) / 2;

		Log.e("Gesture", "Java-Gesture: x,y,s: " + x + "," + y + "," + size);

		float dist = global.gestureSizeAtOne / size;
		// TODO: Potentially remap distance here to obtain greater reach
		// dist = 0.2f;

		x -= global.gestureOffset[0];
		y -= global.gestureOffset[1];

		Log.e("Gesture", "Java-Gesture offsetted: x,y: " + x + "," + y);

		// TODO: Centralize camera calibration
		x /= 540.5454 / 2;
		y /= -539.3325 / 2;

		x *= dist;
		y *= dist;
		float z = -(float)Math.sqrt(dist * dist - x * x - y * y);

		xyz[0] = x;
		xyz[1] = y;
		xyz[2] = z;

		return true;
	}

	public boolean isTrackingEstablished() {
		JNI.Gesture_getTransformationRelative(System.nanoTime(), gestureTransformation);

		// non-tracking marked via all-zero
		return gestureTransformation[8] != 0;
	}
}
