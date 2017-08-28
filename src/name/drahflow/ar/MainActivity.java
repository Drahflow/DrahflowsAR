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

import name.drahflow.ar.geometry.Cube;
import name.drahflow.ar.geometry.Translation;
import name.drahflow.ar.geometry.Constants;
import name.drahflow.ar.geometry.Collection;

public class MainActivity implements ArActivity, GLSurfaceView.Renderer {
	private GlobalState global;

	public void onTouchEvent(MotionEvent e) {
		global.main.switchTo(new MainMenuActivity(global));
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

	@Override
	public void onDrawFrame(GL10 glUnused) {
		// FIXME: This is not optimal; it should use a Choreographer to reduce display latency

		boolean gestured = global.gestureTracker.getPosition(pointerXYZ);
		Log.e("AR", "pointerXYZ: " + pointerXYZ[0] + "," + pointerXYZ[1] + "," + pointerXYZ[2]);
		if(gestured) {
			pointer.setSubgraph(pointerCube);
			pointer.setTranslation(pointerXYZ);
		} else {
			pointer.setSubgraph(null);
		}

		global.view.render(scene);
	}
}
