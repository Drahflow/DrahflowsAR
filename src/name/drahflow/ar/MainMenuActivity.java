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
	private int width;
	private int height;
	private float mouseX, mouseY;
	private Float mouseDownX, mouseDownY;

	public void onTouchEvent(MotionEvent e) {
		Log.e("AR", "TouchEvent: " + e);

		float x = e.getAxisValue(MotionEvent.AXIS_X);
		float y = e.getAxisValue(MotionEvent.AXIS_Y);
		if(x > width / 2) {
			x -= width / 1.82f;
		} else {
			x -= width / 6.2f;
		}

		x -= width / 4;
		y -= height / 2;

		mouseX = x / width / 1.6f;
		mouseY = -y / height / 5.4f;

		if((e.getActionMasked() & MotionEvent.ACTION_DOWN) != 0) {
			mouseDownX = mouseX;
			mouseDownY = mouseY;
		}

		if((e.getActionMasked() & MotionEvent.ACTION_UP) != 0) {
			if(mouseDownX != null && mouseDownY != null) {
				float dx = mouseDownX - mouseX;
				float dy = mouseDownY - mouseY;
				if(dx * dx + dy * dy < 0.01f) {
					for(MenuElement me: menuElements) {
						if(me.isHit(mouseDownX, mouseDownY)) me.onClick();
					}
				}
			}
		}
	}

	public void onPause() {};
	public void onResume() {};

	private GLSurfaceView.Renderer renderer;
	public GLSurfaceView.Renderer getRenderer() {
		return renderer;
	}

	private GlobalState global;

	public MainMenuActivity(GlobalState _global) {
		global = _global;

		renderer = new Renderer();
	}

	abstract class MenuElement {
		float x, y, s;

		protected void draw(Renderer r, float[] color) {
			r.drawRect(x, y, -1f, s, color);
		}

		public boolean isHit(float mx, float my) {
			return mx > x - s && mx < x + s &&
					my > y - s && my > y + s;
		}

		abstract public void draw(Renderer r);
		abstract public void onClick();
	}

	private final float[] RED = new float[] { 1f, 0f, 0f, 0f };
	private final float[] GREEN = new float[] { 0f, 1f, 0f, 0f };

	private MenuElement[] menuElements = new MenuElement[] {
		new MenuElement() {
			{ x = 0f; y = 0.05f; s = 0.01f; }

			public void draw(Renderer r) {
				draw(r, global.cameraTracker.isTrackingEstablished()? GREEN: RED);
			}
			public void onClick() {
				// TODO: (re-)initialize camera tracking
			}
		},
		new MenuElement() {
			{ x = 0f; y = 0.00f; s = 0.01f; }

			public void draw(Renderer r) {
				draw(r, global.gestureTracker.isTrackingEstablished()? GREEN: RED);
			}
			public void onClick() {
				// TODO: (re-)initialize gesture tracking
			}
		},
		new MenuElement() {
			{ x = 0f; y = -0.05f; s = 0.01f; }

			public void draw(Renderer r) {
				draw(r,
						global.cameraTracker.isTrackingEstablished() &&
						global.gestureTracker.isTrackingEstablished()? GREEN: RED);
			}
			public void onClick() {
				// TODO: activate main AR Activity
			}
		}
	};
	
	private class Renderer implements GLSurfaceView.Renderer {
		private FloatBuffer rectPositions;
		private int positionHandle;
		private int texCoordsHandle;

		private float[] viewMatrix = new float[16];
		private float[] modelMatrix = new float[16];
		private float[] projectionMatrix = new float[16];
		private float[] mvpMatrix = new float[16];
		private float[] mvMatrix = new float[16];
		private int mvpMatrixHandle;
		private int colorHandle;

		private int linkedShaderHandle;

		public Renderer() {
			loadModelData();
		}

		private void loadModelData() {
			float[] positions = {
				-1, -1, 0,   1, -1, 0,
				 1, -1, 0,   1,  1, 0,
				 1,  1, 0,  -1,  1, 0,
				-1,  1, 0,  -1, -1, 0,
			};

			rectPositions = ByteBuffer.allocateDirect(positions.length * Utils.BYTES_PER_FLOAT)
					.order(ByteOrder.nativeOrder()).asFloatBuffer();
			rectPositions.put(positions).position(0);
		}

		@Override
		public void onSurfaceCreated(GL10 glUnused, EGLConfig config) {
			GLES20.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			GLES20.glEnable(GLES20.GL_CULL_FACE);
			GLES20.glEnable(GLES20.GL_DEPTH_TEST);

			linkedShaderHandle = Utils.compileShader(getVertexShader(), getFragmentShader(),
					new String[] {"a_Position"});
		}

		@Override
		public void onSurfaceChanged(GL10 glUnused, int _width, int _height) {
			// Set the OpenGL viewport to the same size as the surface.
			width = _width;
			height = _height;

			// Create a new perspective projection matrix. The height will stay the same
			// while the width will vary as per aspect ratio.
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

		@Override
		public void onDrawFrame(GL10 glUnused) {
			final float eyeX = 0.0f;
			final float eyeY = 0.0f;
			final float eyeZ = 0.0f;

			final float lookX = 0.0f;
			final float lookY = 0.0f;
			final float lookZ = -5.0f;

			final float upX = 0.0f;
			final float upY = 1.0f;
			final float upZ = 0.0f;

			// Set program handles for cube drawing.
			mvpMatrixHandle = GLES20.glGetUniformLocation(linkedShaderHandle, "u_MVPMatrix");
			colorHandle = GLES20.glGetUniformLocation(linkedShaderHandle, "u_Color");
			positionHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_Position");
			texCoordsHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_TexCoordinate");

			GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

			Utils.noGlError();

			GLES20.glViewport(0, 0, width / 2, height);
			Matrix.setLookAtM(viewMatrix, 0, eyeX - 0.10f, eyeY, eyeZ, lookX - 0.10f, lookY, lookZ, upX, upY, upZ);
			drawEyeView();
			Utils.noGlError();

			GLES20.glViewport(width / 2, 0, width / 2, height);
			Matrix.setLookAtM(viewMatrix, 0, eyeX - 0.03f, eyeY, eyeZ, lookX - 0.03f, lookY, lookZ, upX, upY, upZ);
			drawEyeView();
			Utils.noGlError();
		}

		private void drawEyeView() {
			drawScene();
		}

		private final float[] MOUSE = new float[] { 0.7f, 0.7f, 0.7f, 0f };
		private void drawScene() {
			for(MenuElement e: menuElements) {
				e.draw(this);
			}

			drawRect(mouseX, mouseY, -1f, 0.002f, MOUSE);
		}

		public void drawRect(float x, float y, float z, float scale, float[] color) {
			Matrix.setIdentityM(modelMatrix, 0);

			GLES20.glUseProgram(linkedShaderHandle);

			// Pass in the position information
			rectPositions.position(0);
			GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
					0, rectPositions);
			GLES20.glEnableVertexAttribArray(positionHandle);

			// Model transformations
			Matrix.translateM(modelMatrix, 0, x, y, z);
			Matrix.scaleM(modelMatrix, 0, scale, scale, scale);

			Matrix.multiplyMM(mvMatrix, 0, viewMatrix, 0, modelMatrix, 0);
			Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, mvMatrix, 0);
			GLES20.glUniformMatrix4fv(mvpMatrixHandle, 1, false, mvpMatrix, 0);
			GLES20.glUniform4fv(colorHandle, 1, color, 0);

			GLES20.glDrawArrays(GLES20.GL_LINES, 0, rectPositions.limit() / 3);
		}

		protected String getVertexShader() {
			final String perPixelVertexShader =
				  "uniform mat4 u_MVPMatrix;      \n"
				+ "uniform vec4 u_Color;          \n"
				+ "attribute vec4 a_Position;     \n"
				+ "varying vec4 v_Color;          \n"

				+ "void main()                                \n"
				+ "{                                          \n"
				+ "   v_Color = u_Color;                      \n"
				+ "   gl_Position = u_MVPMatrix * a_Position; \n"
				+ "}                                          \n";

			return perPixelVertexShader;
		}

		protected String getFragmentShader() {
			final String perPixelFragmentShader =
				  "precision mediump float;\n"
				+ "varying vec4 v_Color;\n"
				+ "void main() {\n"
				+ "   gl_FragColor = v_Color; \n"
				+ "}\n";

			return perPixelFragmentShader;
		}
	}
}
