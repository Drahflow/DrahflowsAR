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

		float x = global.videoHistory.width / 4;
		float y1 = global.videoHistory.height / 4 - 1.0f;
		float y2 = global.videoHistory.height / 4 + 1.0f;

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
		float dist = 1 / global.gestureSizeAtOne;
		// TODO: Potentially remap distance here to obtain greater reach

		// TODO: Centralize camera calibration
		x -= 318.0489 / 2;
		y -= 237.989 / 2;

		x /= 540.5454;
		y /= 539.3325;

		x *= dist;
		y *= dist;
		float z = -(float)Math.sqrt(dist * dist - x * x - y * y);

		global.view.getPose(poseMatrix);

		Matrix.invertM(inversePoseMatrix, 0, poseMatrix, 0);
		final float[] ivp = inversePoseMatrix;
		
		xyz[0] = x * ivp[0] + y * ivp[1] + z * ivp[2] + ivp[3];
		xyz[1] = x * ivp[4] + y * ivp[5] + z * ivp[6] + ivp[7];
		xyz[2] = x * ivp[8] + y * ivp[9] + z * ivp[10] + ivp[11];
		float w = x * ivp[12] + y * ivp[13] + z * ivp[14] + ivp[15];

		xyz[0] /= w;
		xyz[1] /= w;
		xyz[2] /= w;

		return true;
	}

	public boolean isTrackingEstablished() {
		JNI.Gesture_getTransformationRelative(System.nanoTime(), gestureTransformation);

		// non-tracking marked via all-zero
		return gestureTransformation[8] != 0;
	}
}
