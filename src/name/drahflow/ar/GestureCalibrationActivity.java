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
import name.drahflow.ar.geometry.Constants;
import name.drahflow.ar.geometry.Cube;
import name.drahflow.ar.geometry.Collection;

public class GestureCalibrationActivity implements ArActivity, GLSurfaceView.Renderer  {
	private int width;
	private int height;

	interface Stage {
		public void render();
		public void click();
	}
	private Stage currentStage;

	public void onTouchEvent(MotionEvent e) {
		if(e.getActionMasked() == MotionEvent.ACTION_DOWN) {
			currentStage.click();
		}
	}

	public void onKeyEvent(KeyEvent e) { }

	private Stage acquireStage = new Stage() {
		public void click() {
			JNI.Gesture_setMarker(minX(), minY(), maxX(), maxY());
			currentStage = vrStage;
		}

		public void render() {
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
			positionHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_Position");
			texCoordsHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_TexCoordinate");
			texSamplerHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "u_TexImage");

			GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

			Utils.noGlError();

			int[] cameraTexture = bindCameraTexture();

			GLES20.glViewport(0, 0, width / 2, height);
			Matrix.setLookAtM(viewMatrix, 0, eyeX - 0.10f, eyeY, eyeZ, lookX - 0.10f, lookY, lookZ, upX, upY, upZ);
			drawEyeView(cameraTexture[0]);
			Utils.noGlError();

			GLES20.glViewport(width / 2, 0, width / 2, height);
			Matrix.setLookAtM(viewMatrix, 0, eyeX - 0.03f, eyeY, eyeZ, lookX - 0.03f, lookY, lookZ, upX, upY, upZ);
			drawEyeView(cameraTexture[0]);
			Utils.noGlError();

			GLES20.glDeleteTextures(1, cameraTexture, 0);
		}
	};

	private Stage vrStage = new Stage() {
		Cube cube = new Cube(-0.01f, 0f, -0.2f, 0.005f);
		Cube position = new Cube(0f, 0f, 0f, 0.0005f);
		{
			position.setTexture(Constants.YELLOW);
		}
		Collection scene = new Collection(cube, position);
		float[] h = new float[9]; // the gesture transformation in camera coordinates
		float[] pos = new float[3];

		public void click() {
			JNI.Gesture_getTransformationRelative(System.nanoTime(), h);
			if(h[8] == 0) {
				currentStage = acquireStage;
				return;
			}

			float distanceToCamera = (float)Math.sqrt(
				-0.2 * -0.2 +
				-0.01 * -0.01
			);

			float x = global.videoHistory.width / 2;
			float y1 = global.videoHistory.height / 2 - 5.0f;
			float y2 = global.videoHistory.height / 2 + 5.0f;

			// apply transformation to given reference coordinates
			float tx = x * h[0] + y1 * h[1] + h[2];
			float ty = x * h[3] + y1 * h[4] + h[5];
			float tw = x * h[6] + y1 * h[7] + h[8];

			float bx = x * h[0] + y2 * h[1] + h[2];
			float by = x * h[3] + y2 * h[4] + h[5];
			float bw = x * h[6] + y2 * h[7] + h[8];

			tx /= tw;
			ty /= tw;
			bx /= bw;
			by /= bw;

			float size = (float)Math.sqrt(
				(tx - bx) * (tx - bx) +
				(ty - by) * (ty - by)
			);

			float sizeAtOne = size * distanceToCamera;
	
			global.gestureSizeAtOne = sizeAtOne;
			global.gestureOffset = new float[] {
				(tx + bx) / 2, (ty + by) / 2
			};

			global.main.switchTo(new MainMenuActivity(global));
		}

		public void render() {
			cube.setTexture(global.gestureTracker.isTrackingEstablished()? Constants.GREEN: Constants.RED);
			global.gestureTracker.getPosition(pos);
			position.setPosition(-0.01f + pos[0], pos[1], 0f);
			global.view.renderUntracked(scene);
		}
	};

	private int minX() { return (int)(global.videoHistory.width * 0.40f); }
	private int maxX() { return (int)(global.videoHistory.width * 0.60f); }
	private int minY() { return (int)(global.videoHistory.width * 0.45f); }
	private int maxY() { return (int)(global.videoHistory.width * 0.55f); }

	public void onPause() {};
	public void onResume() {};

	public GLSurfaceView.Renderer getRenderer() {
		return this;
	}

	private GlobalState global;

	public GestureCalibrationActivity(GlobalState _global) {
		global = _global;

		loadModelData();

		currentStage = acquireStage;
	}

	private final float[] RED = new float[] { 1f, 0f, 0f, 0f };
	private final float[] GREEN = new float[] { 0f, 1f, 0f, 0f };

	private static FloatBuffer outputTextureBuffer;
	private static float[] outputData;

	private FloatBuffer quadPositions;
	private FloatBuffer quadTexCoords;
	private int positionHandle;
	private int texCoordsHandle;

	private float[] viewMatrix = new float[16];
	private float[] modelMatrix = new float[16];
	private float[] projectionMatrix = new float[16];
	private float[] mvpMatrix = new float[16];
	private float[] mvMatrix = new float[16];
	private int mvpMatrixHandle;
	private int texSamplerHandle;

	private int linkedShaderHandle;

	private void loadModelData() {
		float[] positions = {
			-1, -1, 0,   1, -1, 0,   1,  1, 0,
			-1, 1, 0,   -1, -1, 0,   1,  1, 0,
		};

		quadPositions = ByteBuffer.allocateDirect(positions.length * Utils.BYTES_PER_FLOAT)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
		quadPositions.put(positions).position(0);

		float[] texCoords = {
			0, 1,    1, 1,     1, 0,
			0, 0,    0, 1,     1, 0
		};

		quadTexCoords = ByteBuffer.allocateDirect(texCoords.length * Utils.BYTES_PER_FLOAT)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
		quadTexCoords.put(texCoords).position(0);
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

		global.view.surfaceChanged(width, height);
	}

	@Override
	public void onDrawFrame(GL10 glUnused) {
		currentStage.render();
	}

	private void drawEyeView(int cameraTexture) {
		drawScene(cameraTexture);
	}

	private void drawScene(int cameraTexture) {
		drawQuad(RED, cameraTexture);
	}

	private int[] bindCameraTexture() {
		final int width = global.videoHistory.width;
		final int height = global.videoHistory.height;

		int[] someTexs = new int[1];
		GLES20.glGenTextures(1, someTexs, 0);
		int tmpTex = someTexs[0];

		if(outputTextureBuffer == null) {
			outputTextureBuffer = ByteBuffer.allocateDirect(4 * width * height * Utils.BYTES_PER_FLOAT)
					.order(ByteOrder.nativeOrder()).asFloatBuffer();
			outputData = new float[width * height * 4];
		}

		// float[] intensities = global.videoHistory.getLastFrame().getIntensities();
		float[] intensities = global.cameraTracker.debugImage;

		for(int i = 0; i < width * height; ++i) {
			outputData[i * 4 + 0] = intensities[i];
			outputData[i * 4 + 1] = intensities[i];
			outputData[i * 4 + 2] = intensities[i];
		}

		int minX = GestureCalibrationActivity.this.minX();
		int maxX = GestureCalibrationActivity.this.maxX();
		int minY = GestureCalibrationActivity.this.minY();
		int maxY = GestureCalibrationActivity.this.maxY();
		for(int line_width = 0; line_width < 4; ++line_width) {
			for(int ty = minY; ty < maxY; ++ty) {
				int leftI = (minX - line_width + ty * width) * 4;
				int rightI = (maxX + line_width + ty * width) * 4;

				outputData[leftI] = 1;
				outputData[rightI] = 1;
			}

			int targetX = width / 2;
			int targetY = (int)(height * 0.4f);
			outputData[(targetX + line_width + targetY * width) * 4 + 1] = 1;
		}

		outputTextureBuffer.position(0);
		outputTextureBuffer.put(outputData);

		outputTextureBuffer.position(0);
		GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, tmpTex);
		GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_RGBA, width, height, 0, GLES30.GL_RGBA, GLES30.GL_FLOAT, outputTextureBuffer);
		GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
		GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);

		return someTexs;
	}

	public void drawQuad(float[] color, int cameraTexture) {
		Matrix.setIdentityM(modelMatrix, 0);

		GLES20.glUseProgram(linkedShaderHandle);

		// Pass in the position information
		quadPositions.position(0);
		GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
				0, quadPositions);
		GLES20.glEnableVertexAttribArray(positionHandle);

		// Pass in texture information
		quadTexCoords.position(0);
		GLES20.glVertexAttribPointer(texCoordsHandle, 2, GLES20.GL_FLOAT, false,
				0, quadTexCoords);
		GLES20.glEnableVertexAttribArray(texCoordsHandle);

		// Model transformations
		Matrix.translateM(modelMatrix, 0, -0.06f, 0f, -1f);
		Matrix.scaleM(modelMatrix, 0, 0.12f, 0.06f, 0.06f);

		Matrix.multiplyMM(mvMatrix, 0, viewMatrix, 0, modelMatrix, 0);
		Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, mvMatrix, 0);
		GLES20.glUniformMatrix4fv(mvpMatrixHandle, 1, false, mvpMatrix, 0);

		GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
		GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, cameraTexture);
		GLES20.glUniform1i(texSamplerHandle, 0);

		GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, quadPositions.limit() / 3);
	}

	protected String getVertexShader() {
		final String perPixelVertexShader =
				"uniform mat4 u_MVPMatrix;      \n"
			+ "attribute vec4 a_Position;     \n"
			+ "attribute vec2 a_TexCoordinate;\n"
			+ "varying vec4 v_Color;          \n"
			+ "varying vec2 v_TexCoordinate;  \n"

			+ "void main()                                \n"
			+ "{                                          \n"
			+ "   v_TexCoordinate = a_TexCoordinate;      \n"
			+ "   gl_Position = u_MVPMatrix * a_Position; \n"
			+ "}                                          \n";

		return perPixelVertexShader;
	}

	protected String getFragmentShader() {
		final String perPixelFragmentShader =
				"precision mediump float;\n"
			+ "uniform sampler2D u_TexImage;\n"
			+ "varying vec2 v_TexCoordinate;\n"
			+ "void main() {\n"
			+ "   gl_FragColor = texture2D(u_TexImage, v_TexCoordinate); \n"
			+ "}\n";

		return perPixelFragmentShader;
	}
}
