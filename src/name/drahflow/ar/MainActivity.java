package name.drahflow.ar;

import javax.microedition.khronos.opengles.GL10;
import javax.microedition.khronos.egl.EGLConfig;
import android.opengl.GLSurfaceView;
import android.opengl.GLES20;
import android.opengl.GLES30;
import android.opengl.GLES11Ext;
import android.opengl.Matrix;
import android.opengl.GLUtils;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import android.util.Log;
import android.view.MotionEvent;
import android.view.KeyEvent;

import name.drahflow.ar.geometry.Cube;
import name.drahflow.ar.geometry.Translation;
import name.drahflow.ar.geometry.Constants;
import name.drahflow.ar.geometry.Collection;

public class MainActivity implements ArActivity, GLSurfaceView.Renderer {
	private GlobalState global;
	private float[] poseMatrix = new float[16];
	private float[] inversePoseMatrix = new float[16];

	public void onTouchEvent(MotionEvent e) {
		global.main.switchTo(new MainMenuActivity(global));
	}

	public void onKeyEvent(KeyEvent e) {
		if(e.getAction() == KeyEvent.ACTION_DOWN) {
			switch(e.getKeyCode()) {
				case KeyEvent.KEYCODE_DPAD_CENTER:
					keypadCursor = !keypadCursor;
					keypadCursorDistance = 0.2f;
					break;
				case KeyEvent.KEYCODE_DPAD_UP:
					keypadCursorDistance *= 1.02;
					break;
				case KeyEvent.KEYCODE_DPAD_DOWN:
					keypadCursorDistance /= 1.02;
					break;
			}
		}
	}

	public void onPause() {};
	public void onResume() {};

	public GLSurfaceView.Renderer getRenderer() {
		return this;
	}

	public MainActivity(GlobalState global) {
		this.global = global;

		pointerCube = new Cube(0, 0, 0, 0.001f);
		pointerCube.setTexture(Constants.YELLOW);

		pointer = new Translation();
		pointer.setSubgraph(pointerCube);

		Cube cube1 = new Cube(0, 0, -0.5f, 0.005f);
		cube1.setTexture(Constants.GREEN);

		Cube cube2 = new Cube(0, 0, -0.25f, 0.005f);
		cube2.setTexture(Constants.RED);

		scene = new Collection(cube1, cube2, pointer);
	}

	@Override public void onSurfaceCreated(GL10 glUnused, EGLConfig config) { }

	@Override
	public void onSurfaceChanged(GL10 glUnused, int width, int height) {
		global.view.surfaceChanged(width, height);
	}

	private Cube pointerCube;
	private Translation pointer;
	private float[] pointerXYZ = new float[3];
	private Collection scene;

	private boolean keypadCursor = false;
	private float keypadCursorDistance;

	@Override
	public void onDrawFrame(GL10 glUnused) {
		// FIXME: This is not optimal; it should use a Choreographer to reduce display latency

		global.view.updatePoseMatrix();

		float x, y, z;
		boolean activeCursor = false;
		if(keypadCursor) {
			activeCursor = true;
			// FIXME: Reverse camera view here
			x = -global.cameraDistance;
			y = 0;
			z = -keypadCursorDistance;
		} else {
			activeCursor = global.gestureTracker.getPosition(pointerXYZ);
			x = pointerXYZ[0];
			y = pointerXYZ[1];
			z = pointerXYZ[2];
		}

		if(activeCursor) {
			global.view.getCachedPose(poseMatrix);
			Matrix.invertM(inversePoseMatrix, 0, poseMatrix, 0);
			final float[] ivp = inversePoseMatrix;
			
			pointerXYZ[0] = x * ivp[0] + y * ivp[4] + z * ivp[8] + ivp[12];
			pointerXYZ[1] = x * ivp[1] + y * ivp[5] + z * ivp[9] + ivp[13];
			pointerXYZ[2] = x * ivp[2] + y * ivp[6] + z * ivp[10] + ivp[14];
			float w = x * ivp[3] + y * ivp[7] + z * ivp[11] + ivp[15];

			pointerXYZ[0] /= w;
			pointerXYZ[1] /= w;
			pointerXYZ[2] /= w;

			Log.e("AR", "pointerXYZ: " + pointerXYZ[0] + "," + pointerXYZ[1] + "," + pointerXYZ[2]);
			pointer.setSubgraph(pointerCube);
			pointer.setTranslation(pointerXYZ);
		} else {
			pointer.setSubgraph(null);
		}

		global.view.renderWithCachedPose(scene);
	}
}
