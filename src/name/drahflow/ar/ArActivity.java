package name.drahflow.ar;

import android.opengl.GLSurfaceView;
import android.view.MotionEvent;

public interface ArActivity {
	public GLSurfaceView.Renderer getRenderer();
	public void onTouchEvent(MotionEvent e);
	public void onPause();
	public void onResume();
}
