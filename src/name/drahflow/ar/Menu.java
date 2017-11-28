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
import java.util.ArrayList;

public class Menu {
	private int width;
	private int height;
	private float mouseX, mouseY;
	private Float mouseDownX, mouseDownY;

	public void onTouchEvent(MotionEvent e) {
		Log.e("AR", "TouchEvent: " + e);

		float x = e.getAxisValue(MotionEvent.AXIS_X);
		float y = e.getAxisValue(MotionEvent.AXIS_Y);
		if(x > width / 2) {
			x -= 275f;
		} else {
			x -= 30f;
		}

		x -= width / 4;
		y -= height / 1.90;

		mouseX = x / width / 1.6f;
		mouseY = -y / height / 5.4f;

		if((e.getActionMasked() == MotionEvent.ACTION_DOWN)) {
			mouseDownX = mouseX;
			mouseDownY = mouseY;
		}

		if((e.getActionMasked() == MotionEvent.ACTION_UP)) {
			if(mouseDownX != null && mouseDownY != null) {
				float dx = mouseDownX - mouseX;
				float dy = mouseDownY - mouseY;

				if(dx * dx + dy * dy < 0.01f) {
					for(Element me: menuElements) {
						if(me.isHit(mouseDownX, mouseDownY)) {
							me.onClick();
							Log.e("AR", "Hit: " + me);
						}
					}
				}
			}
		}
	}

	private Renderer renderer;
	public Renderer getRenderer() {
		return renderer;
	}

	public Menu() {
		renderer = new Renderer();
	}

	protected void draw(Renderer r) {
		// add global elements in subclasses if desired
	}

	abstract static class Element {
		float x, y, s;
		String title;

		protected void draw(Renderer r, float[] color) {
			r.drawRect(x, y, -1f, s, color);
			if(title != null) {
				r.drawText(Utils.renderText(title), x, y - 0.02f, -1f, s * 3);
			}
		}

		public boolean isHit(float mx, float my) {
			return mx > x - s && mx < x + s &&
					my > y - s && my < y + s;
		}

		abstract public void draw(Renderer r);
		abstract public void onClick();
	}

	public static final float[] RED = new float[] { 1f, 0f, 0f, 0f };
	public static final float[] GREEN = new float[] { 0f, 1f, 0f, 0f };

	private ArrayList<Element> menuElements = new ArrayList<>();
	public void add(Element item) {
		menuElements.add(item);
	}
	
	public class Renderer implements GLSurfaceView.Renderer {
		private FloatBuffer rectPositions;
		private FloatBuffer quadPositions;
		private int positionHandle;
		private int texCoordsHandle;

		private float[] viewMatrix = new float[16];
		private float[] modelMatrix = new float[16];
		private float[] projectionMatrix = new float[16];
		private float[] mvpMatrix = new float[16];
		private float[] mvMatrix = new float[16];
		private int mvpMatrixHandle;
		private int colorHandle;

		private int linkedFlatShaderHandle;
		private int linkedTexturedShaderHandle;

		public Renderer() {
			loadModelData();
		}

		private void loadModelData() {
			float[] positions = {
				// GL_LINES version
				-1, -1, 0,   1, -1, 0,
				 1, -1, 0,   1,  1, 0,
				 1,  1, 0,  -1,  1, 0,
				-1,  1, 0,  -1, -1, 0,
			};

			rectPositions = ByteBuffer.allocateDirect(positions.length * Utils.BYTES_PER_FLOAT)
					.order(ByteOrder.nativeOrder()).asFloatBuffer();
			rectPositions.put(positions).position(0);

			positions = new float[] {
				// GL_TRIANGLES version
				-1, -1, 0,
				 1, -1, 0,
				 1,  1, 0,

				-1,  1, 0,
				-1, -1, 0,
				 1,  1, 0,
			};

			quadPositions = ByteBuffer.allocateDirect(positions.length * Utils.BYTES_PER_FLOAT)
					.order(ByteOrder.nativeOrder()).asFloatBuffer();
			quadPositions.put(positions).position(0);
		}

		@Override
		public void onSurfaceCreated(GL10 glUnused, EGLConfig config) {
			GLES20.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			GLES20.glEnable(GLES20.GL_CULL_FACE);
			GLES20.glEnable(GLES20.GL_DEPTH_TEST);

			linkedFlatShaderHandle  = Utils.compileShader(getVertexShader(), getFragmentShader(),
					new String[] {"a_Position"});

			linkedTexturedShaderHandle = Utils.compileShader(
					getTexturedVertexShader(), getTexturedFragmentShader(),
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
			GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);
			
			onDrawOverlay();
		}

		public void onDrawOverlay() {
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
			colorHandle = GLES20.glGetUniformLocation(linkedFlatShaderHandle , "u_Color");
			positionHandle = GLES20.glGetAttribLocation(linkedFlatShaderHandle , "a_Position");

			Utils.noGlError();

			GLES20.glViewport(0, 0, width / 2, height);
			Matrix.setLookAtM(viewMatrix, 0, eyeX - 0.035f, eyeY, eyeZ, lookX - 0.035f, lookY, lookZ, upX, upY, upZ);
			drawEyeView();
			Utils.noGlError();

			GLES20.glViewport(width / 2, 0, width / 2, height);
			Matrix.setLookAtM(viewMatrix, 0, eyeX + 0.035f, eyeY, eyeZ, lookX + 0.035f, lookY, lookZ, upX, upY, upZ);
			drawEyeView();
			Utils.noGlError();
		}

		private void drawEyeView() {
			drawScene();
		}

		private final float[] MOUSE = new float[] { 0.7f, 0.7f, 0.7f, 0f };
		private void drawScene() {
			Menu.this.draw(this);

			for(Element e: menuElements) {
				e.draw(this);
			}

			drawRect(mouseX, mouseY, -1f, 0.002f, MOUSE);
		}

		public void drawRect(float x, float y, float z, float scale, float[] color) {
			// Model transformations
			Matrix.setIdentityM(modelMatrix, 0);
			Matrix.translateM(modelMatrix, 0, x, y, z);
			Matrix.scaleM(modelMatrix, 0, scale, scale, scale);
			Matrix.multiplyMM(mvMatrix, 0, viewMatrix, 0, modelMatrix, 0);
			Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, mvMatrix, 0);

			// Pass in the position information
			rectPositions.position(0);
			GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
					0, rectPositions);
			GLES20.glEnableVertexAttribArray(positionHandle);

			GLES20.glUseProgram(linkedFlatShaderHandle);
			mvpMatrixHandle = GLES20.glGetUniformLocation(linkedFlatShaderHandle , "u_MVPMatrix");
			GLES20.glUniformMatrix4fv(mvpMatrixHandle, 1, false, mvpMatrix, 0);
			GLES20.glUniform4fv(colorHandle, 1, color, 0);

			GLES20.glDrawArrays(GLES20.GL_LINES, 0, rectPositions.limit() / 3);
		}

		public void drawText(Utils.TextInfo text, float x, float y, float z, float scale) {
			// Model transformations
			Matrix.setIdentityM(modelMatrix, 0);
			Matrix.translateM(modelMatrix, 0, x, y, z);
			Matrix.scaleM(modelMatrix, 0, scale * text.width / 160, scale * text.height / 160, scale);
			Matrix.multiplyMM(mvMatrix, 0, viewMatrix, 0, modelMatrix, 0);
			Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, mvMatrix, 0);

			// Pass in the position information
			quadPositions.position(0);
			GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
					0, quadPositions);
			GLES20.glEnableVertexAttribArray(positionHandle);

			GLES20.glUseProgram(linkedTexturedShaderHandle);
			mvpMatrixHandle = GLES20.glGetUniformLocation(linkedTexturedShaderHandle , "u_MVPMatrix");
			GLES20.glUniformMatrix4fv(mvpMatrixHandle, 1, false, mvpMatrix, 0);

			GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, text.texture);
			int textureHandle = GLES20.glGetUniformLocation(linkedTexturedShaderHandle , "u_Texture");
			GLES20.glUniform1i(textureHandle, 0);

			GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, quadPositions.limit() / 3);
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

		protected String getTexturedVertexShader() {
			final String perPixelVertexShader =
				  "uniform mat4 u_MVPMatrix;      \n"
				+ "attribute vec4 a_Position;     \n"
				+ "varying vec2 v_TexCoord;          \n"

				+ "void main()                                \n"
				+ "{                                          \n"
				+ "   gl_Position = u_MVPMatrix * a_Position; \n"
				+ "   v_TexCoord = vec2(a_Position.x + 1.0, -a_Position.y + 1.0) / 2.0;\n"
				+ "}                                          \n";

			return perPixelVertexShader;
		}

		protected String getTexturedFragmentShader() {
			final String perPixelFragmentShader =
				  "precision mediump float;\n"
				+ "uniform sampler2D u_Texture;\n"
				+ "varying vec2 v_TexCoord;\n"
				+ "void main() {\n"
				+ "   gl_FragColor = texture2D(u_Texture, v_TexCoord); \n"
				+ "}\n";

			return perPixelFragmentShader;
		}
	}
}
