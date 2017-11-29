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

import name.drahflow.ar.geometry.PointerEvent;
import name.drahflow.ar.geometry.Geometry;
import name.drahflow.ar.geometry.Cube;
import name.drahflow.ar.geometry.Translation;
import name.drahflow.ar.geometry.Scaling;
import name.drahflow.ar.geometry.Constants;
import name.drahflow.ar.geometry.Collection;
import name.drahflow.ar.geometry.SpaceMenu;
import name.drahflow.ar.geometry.SpaceSelector;
import name.drahflow.ar.geometry.OverlayText;

public class MainActivity implements ArActivity, GLSurfaceView.Renderer {
	private GlobalState global;
	private float[] poseMatrix = new float[16];
	private float[] inversePoseMatrix = new float[16];

	OverlayText[] overlays;

	public void onTouchEvent(MotionEvent e) {
		if((e.getActionMasked() == MotionEvent.ACTION_DOWN)) {
			if(overlays[0].isShowing()) {
				global.main.switchTo(new MainMenuActivity(global));
			}
		}

		synchronized(MainActivity.this) {
			for(OverlayText overlay: overlays) overlay.show();
		}
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

		pointerCube = new Cube(0, 0, 0, 0.0001f);
		pointerCube.setTexture(Constants.YELLOW);

		pointer = new Translation();
		pointer.setSubgraph(pointerCube);

		Cube cube1 = new Cube(0, 0, -0.5f, 0.005f);
		cube1.setTexture(Constants.GREEN);

		Cube cube2 = new Cube(0, 0, -0.25f, 0.005f);
		cube2.setTexture(Constants.RED);

		spaceMenu = new SpaceMenu(global);
		scene = new Collection(cube1, cube2, pointer, spaceMenu);

		overlays = new OverlayText[] {
			new OverlayText(global, "Use center key to enable 3D cursor.", -0.02f, 0.01f),
			new OverlayText(global, "Use up/down to shift cursor distance.", -0.02f, 0.005f),
			new OverlayText(global, "Hold cursor still to activate.", -0.02f, 0),
			new OverlayText(global, "Click touchpad again to exit.", -0.02f, -0.005f)
		};
		for(OverlayText overlay: overlays) scene.add(overlay);

		setupMenu();
	}

	private void setupMenu() {
		spaceMenu.setTitle(0, "VNC");
		spaceMenu.setTitle(1, "Nothing configured");
		spaceMenu.setTitle(2, "Nothing configured");
		spaceMenu.setTitle(3, "Nothing configured");
		spaceMenu.setTitle(4, "Nothing configured");
		spaceMenu.setTitle(5, "Nothing configured");
		spaceMenu.setAction(0, 0, new SpaceMenu.ActionCallback() {
			@Override public String getTitle() { return "localhost:5900"; }
			@Override public void selected(
				final float[] activationPose,
				final float x, final float y, final float z
			) {
				global.main.runOnUiThread(new Runnable() {
					@Override public void run() {
						synchronized(MainActivity.this) {
							scene.add(new SpaceSelector(global, activationPose, x, y, z,
								new SpaceSelector.ActionCallback() {
									public void selected(float[] activationPose,
											final float sx, final float sy, final float sz,
											final float ex, final float ey, final float ez) {
										global.main.runOnUiThread(new Runnable() {
											@Override public void run() {
												Geometry vnc = new name.drahflow.ar.apps.VNC.Window(
														global.main, "localhost", 5900);
												synchronized(MainActivity.this) {
													scene.add(
		 												new Translation(sx, sy, sz,
		 													new Scaling(ex - sx, ey - sy, ez - sz,
		 														vnc)));
												}
											}
										});
									}
								}
							));
						}
					}
				});
			}
		});

		spaceMenu.setAction(0, 1, new SpaceMenu.ActionCallback() {
			@Override public String getTitle() { return "Test Cube"; }
			@Override public void selected(
				final float[] activationPose,
				final float x, final float y, final float z
			) {
				global.main.runOnUiThread(new Runnable() {
					@Override public void run() {
						Geometry vnc = new Cube(0, 0, 0, 0.005f);
						synchronized(MainActivity.this) {
							scene.add(new Translation(x, y, z, vnc));
						}
					}
				});
			}
		});
	}

	@Override public void onSurfaceCreated(GL10 glUnused, EGLConfig config) { }

	@Override
	public void onSurfaceChanged(GL10 glUnused, int width, int height) {
		global.view.surfaceChanged(width, height);
	}

	private Cube pointerCube;
	private SpaceMenu spaceMenu;
	private Translation pointer;
	private Collection scene;
	private Geometry focusedElement = null;
	private PointerEvent pointerEvent = new PointerEvent();
	private float[] pointerXYZ = new float[3];

	private boolean keypadCursor = false;
	private float keypadCursorDistance;

	@Override
	public synchronized void onDrawFrame(GL10 glUnused) {
		// FIXME: This is not optimal; it should use a Choreographer to reduce display latency

		global.view.updatePoseMatrix();

		float x, y, z;
		boolean activeCursor = false;
		if(keypadCursor) {
			activeCursor = true;
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
			pointerCube.setTexture(global.cameraTracker.hasGoodTracking()? Constants.YELLOW: Constants.RED);

			global.view.getCachedPose(poseMatrix);
			Matrix.invertM(inversePoseMatrix, 0, poseMatrix, 0);
			final float[] ivp = inversePoseMatrix;
			
			pointerEvent.x = x * ivp[0] + y * ivp[4] + z * ivp[8] + ivp[12];
			pointerEvent.y = x * ivp[1] + y * ivp[5] + z * ivp[9] + ivp[13];
			pointerEvent.z = x * ivp[2] + y * ivp[6] + z * ivp[10] + ivp[14];
			float w = x * ivp[3] + y * ivp[7] + z * ivp[11] + ivp[15];

			pointerEvent.x /= w;
			pointerEvent.y /= w;
			pointerEvent.z /= w;

			pointer.setSubgraph(pointerCube);
			pointer.setTranslation(pointerEvent.x, pointerEvent.y, pointerEvent.z);

			pointerEvent.timestamp = System.nanoTime();
			pointerEvent.focusedOn = focusedElement;
			pointerEvent.active = activeCursor;

			focusedElement = scene.onPointer(pointerEvent);
		} else {
			pointer.setSubgraph(null);

			if(focusedElement != null) {
				pointerEvent.active = false;
				scene.onPointer(pointerEvent);
			}

			focusedElement = null;
		}

		global.view.renderWithCachedPose(scene);
	}
}
