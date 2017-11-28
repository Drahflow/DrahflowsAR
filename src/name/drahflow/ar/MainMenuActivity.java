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

public class MainMenuActivity implements ArActivity {
	public void onTouchEvent(MotionEvent e) {
		menu.onTouchEvent(e);
	}

	public void onKeyEvent(KeyEvent e) {
		// TODO: Could make menu navigatable with keypad
	}

	public void onPause() {};
	public void onResume() {};

	public GLSurfaceView.Renderer getRenderer() {
		return menu.getRenderer();
	}

	private GlobalState global;
	private Menu menu;

	public MainMenuActivity(GlobalState _global) {
		global = _global;

		menu = new Menu() {
			@Override protected void draw(Menu.Renderer r) {
				r.drawText(Utils.renderText("Use touchpad to select action."), 0, 0.08f, -1, 0.03f);
			}
		};

		menu.add(new Menu.Element() {
			{ x = -0.05f; y = 0.05f; s = 0.01f; title = "Reset pose tracking"; }

			public void draw(Menu.Renderer r) {
				draw(r, global.cameraTracker.hasGoodTracking()? Menu.GREEN: Menu.RED);
				if(!global.cameraTracker.hasGoodTracking()) {
					r.drawText(Utils.renderText("Move head left + right until green"),
							x, y - 0.03f, -1f, s * 3);
				}
			}
			public void onClick() {
				global.cameraTracker.initialize();
			}
		});
		menu.add(new Menu.Element() {
			{ x = 0.05f; y = 0.05f; s = 0.01f; title = "AR calibration"; }

			public void draw(Menu.Renderer r) {
				draw(r, Menu.RED);
			}
			public void onClick() {
				global.main.switchTo(new ScaleCalibrationActivity(global));
			}
		});
		menu.add(new Menu.Element() {
			{ x = 0f; y = 0.00f; s = 0.01f; title = "Hand tracking"; }

			public void draw(Menu.Renderer r) {
				draw(r, global.gestureTracker.isTrackingEstablished()? Menu.GREEN: Menu.RED);
			}
			public void onClick() {
				global.main.switchTo(new GestureCalibrationActivity(global));
			}
		});
		menu.add(new Menu.Element() {
			{ x = 0f; y = -0.05f; s = 0.01f; title = "Switch to AR"; }

			public void draw(Menu.Renderer r) {
				draw(r,
						global.cameraTracker.hasGoodTracking() &&
						global.gestureTracker.isTrackingEstablished()? Menu.GREEN: Menu.RED);
			}
			public void onClick() {
				global.main.switchTo(new MainActivity(global));
			}
		});
	};
}
