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

public class MainMenuActivity implements ArActivity {
	public void onTouchEvent(MotionEvent e) {
		menu.onTouchEvent(e);
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

		menu = new Menu();
		menu.add(new Menu.Element() {
			{ x = -0.05f; y = 0.05f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, global.cameraTracker.hasGoodTracking()? Menu.GREEN: Menu.RED);
			}
			public void onClick() {
				global.cameraTracker.initialize();
			}
		});
		menu.add(new Menu.Element() {
			{ x = 0.05f; y = 0.05f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, Menu.RED);
			}
			public void onClick() {
				global.main.switchTo(new ScaleCalibrationActivity(global));
			}
		});
		menu.add(new Menu.Element() {
			{ x = 0f; y = 0.00f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, global.gestureTracker.isTrackingEstablished()? Menu.GREEN: Menu.RED);
			}
			public void onClick() {
				global.main.switchTo(new GestureCalibrationActivity(global));
			}
		});
		menu.add(new Menu.Element() {
			{ x = 0f; y = -0.05f; s = 0.01f; }

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
