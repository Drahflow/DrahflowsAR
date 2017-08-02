package name.drahflow.ar;

import android.app.Activity;
import android.app.ActivityManager;
import android.content.Context;
import android.content.pm.ConfigurationInfo;
import android.os.Bundle;
import android.os.Environment;
import javax.microedition.khronos.egl.EGLConfig;
import android.app.Activity;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.Log;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.SurfaceTexture;
import android.view.View;
import android.view.Surface;
import android.view.Window;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraManager;
import com.epson.moverio.btcontrol.DisplayControl;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.FileOutputStream;
import javax.microedition.khronos.egl.EGL10;
import javax.microedition.khronos.egl.EGLDisplay;
import android.view.MotionEvent;

public class DrahflowsAR extends Activity {
	private GLSurfaceView mainView;
	private ArActivity activity;
	private GlobalState global = new GlobalState();

	private static final int camera_width = 640;
	private static final int camera_height = 480;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		global.main = this;

		// disable non-3D elements
		requestWindowFeature(Window.FEATURE_NO_TITLE);
		View view = getWindow().getDecorView();
		view.setSystemUiVisibility(View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);

		// Create our Preview view and set it as the content of our
		// Activity
		final ActivityManager activityManager = (ActivityManager) getSystemService(Context.ACTIVITY_SERVICE);
		final ConfigurationInfo configurationInfo = activityManager.getDeviceConfigurationInfo();
		final boolean supportsEs2 = configurationInfo.reqGlEsVersion >= 0x20000;
		if(!supportsEs2) {
			throw new RuntimeException("ES2 not supported, this is hopeless");
		}

		final int width = camera_width;
		final int height = camera_height;
		global.videoHistory = new VideoHistory(width, height, camera_width, camera_height, 30l * 1000l * 1000l * 1000l);
		global.cameraTracker = new CameraTracker((CameraManager)getSystemService(CAMERA_SERVICE), width, height, global.videoHistory);
		global.sensorTracker = new SensorTracker((SensorManager)getSystemService(SENSOR_SERVICE), global.videoHistory, global.cameraTracker);
		global.gestureTracker = new GestureTracker();

		switchTo(new MainMenuActivity(global));
	}

	public void switchTo(ArActivity new_activity) {
		if(activity != null) activity.onPause();

		activity = new_activity;
		activity.onResume();

		mainView = new GLSurfaceView(this) {
			public boolean onTouchEvent(MotionEvent e) {
				Log.e("AR", "onTouchEvent: " + e);
				activity.onTouchEvent(e);
				return true;
			}
			public boolean onTrackballEvent(MotionEvent e) {
				Log.e("AR", "onTrackballEvent: " + e);
				return false;
			}
		};
		mainView.setEGLContextClientVersion(2);
		mainView.setEGLConfigChooser(new GLSurfaceView.EGLConfigChooser() {
			public EGLConfig chooseConfig(EGL10 egl, EGLDisplay display) {
				int[] ret = new int[1];
				EGLConfig[] configs = new EGLConfig[64];

				int[] configSpec = {
					EGL10.EGL_RED_SIZE, 8,
					EGL10.EGL_GREEN_SIZE, 8,
					EGL10.EGL_BLUE_SIZE, 8,
					EGL10.EGL_ALPHA_SIZE, 8,
					EGL10.EGL_DEPTH_SIZE, 16,
					EGL10.EGL_RENDERABLE_TYPE, 4,
					EGL10.EGL_SAMPLE_BUFFERS, 1,
					EGL10.EGL_SAMPLES, 4,
					EGL10.EGL_NONE
				};

				egl.eglChooseConfig(display, configSpec, configs, configs.length, ret);

				Log.e("AR", "Available GL configs: " + ret[0]);
				
				return configs[0];
			}
		});
		setContentView(mainView);

		mainView.setRenderer(activity.getRenderer());
	}

	@Override
	protected void onResume() {
		// Ideally a game should implement onResume() and onPause()
		// to take appropriate action when the activity looses focus
		super.onResume();
		mainView.onResume();
		new DisplayControl(this).setMode(DisplayControl. DISPLAY_MODE_3D, false);

		global.onResume();
		activity.onResume();
	}

	@Override
	protected void onPause() {
		// Ideally a game should implement onResume() and onPause()
		// to take appropriate action when the activity looses focus
		super.onPause();
		mainView.onPause();
		new DisplayControl(this).setMode(DisplayControl. DISPLAY_MODE_2D, false);

		activity.onPause();
		global.onPause();
	}
}
