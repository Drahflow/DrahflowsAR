package name.drahflow.ar.geometry;

import android.opengl.Matrix;
import android.opengl.GLES20;

import name.drahflow.ar.GlobalState;
import name.drahflow.ar.Utils;

public class View {
	private GlobalState global;

	private int surfaceWidth, surfaceHeight;
	private float[] shiftMatrix = new float[16];
	private float[] projectionMatrix = new float[16];
	private float[] lookAtMatrix = new float[16];
	private float[] viewMatrix = new float[16];
	private float[] vpMatrix = new float[16];
	private float[] vpsMatrix = new float[16];

	public View(GlobalState global) {
		this.global = global;
	}

	public void render(Geometry g) {
		GLES20.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		GLES20.glEnable(GLES20.GL_CULL_FACE);
		GLES20.glEnable(GLES20.GL_DEPTH_TEST);
		GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

		float[] pose = new float[7];
		global.cameraTracker.getTransformationAt(System.nanoTime() + global.displayLag, pose);

		final float qi = pose[3];
		final float qj = -pose[4];
		final float qk = -pose[5];
		final float qr = pose[6];
		final float[] rotationMatrix = new float[] {
			1f - 2f*(qj*qj + qk*qk), 2f*(qi*qj - qk*qr), 2f*(qi*qk + qj*qr), 0f,
			2f*(qi*qj + qk*qr), 1f - 2f*(qi*qi + qk*qk), 2f*(qj*qk - qi*qr), 0f,
			2f*(qi*qk - qj*qr), 2f*(qj*qk + qi*qr), 1f - 2f*(qi*qi + qj*qj), 0f,
			0f,                 0f,                 0f,                      1f
		};

		Matrix.translateM(rotationMatrix, 0,
				global.vrScale * -pose[0],
				global.vrScale * pose[1],
				global.vrScale * pose[2]);

		GLES20.glViewport(0, 0, surfaceWidth / 2, surfaceHeight);
		Utils.noGlError();

		Matrix.setIdentityM(shiftMatrix, 0);
		Matrix.translateM(shiftMatrix, 0, -global.eyeShift / 2, 0, 0);
		Matrix.setLookAtM(lookAtMatrix, 0,
				-global.eyeDistance / 2, 0f, 0f,
				-global.eyeDistance / 2, 0f, -5f,
				0f, 1f, 0f);
		Utils.noGlError();
		Matrix.multiplyMM(viewMatrix, 0, lookAtMatrix, 0, rotationMatrix, 0);
		Matrix.multiplyMM(vpMatrix, 0, projectionMatrix, 0, viewMatrix, 0);
		Matrix.multiplyMM(vpsMatrix, 0, shiftMatrix, 0, vpMatrix, 0);
		g.render(vpsMatrix);
		Utils.noGlError();

		GLES20.glViewport(surfaceWidth / 2, 0, surfaceWidth / 2, surfaceHeight);
		Utils.noGlError();
		Matrix.setIdentityM(shiftMatrix, 0);
		Matrix.translateM(shiftMatrix, 0, global.eyeShift / 2, 0, 0);
		Matrix.setLookAtM(lookAtMatrix, 0,
				global.eyeDistance / 2, 0f, 0f,
				global.eyeDistance / 2, 0f, -5f,
				0f, 1f, 0f);
		Matrix.multiplyMM(viewMatrix, 0, lookAtMatrix, 0, rotationMatrix, 0);
		Matrix.multiplyMM(vpMatrix, 0, projectionMatrix, 0, viewMatrix, 0);
		Matrix.multiplyMM(vpsMatrix, 0, shiftMatrix, 0, vpMatrix, 0);
		g.render(vpsMatrix);
		Utils.noGlError();
	}
	
	public void surfaceChanged(int width, int height) {
		surfaceWidth = width;
		surfaceHeight = height;

		final float ratio = (float) width / height;
		final float zoom = 111f;
		final float left = -ratio / zoom;
		final float right = ratio / zoom;
		final float bottom = -1.0f / zoom;
		final float top = 1.0f / zoom;
		final float near = 0.1f;
		final float far = 50.0f;

		Matrix.frustumM(projectionMatrix, 0, left, right, bottom, top, near, far);
	}
}
