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

public class ScaleCalibrationActivity implements ArActivity, GLSurfaceView.Renderer {
	private GlobalState global;
	private Menu menu;

	interface CalibratableScale {
		public void delta(float f);
		public void render(float[] pose);
	}

	private CalibratableScale currentScale;
	private CalibratableScale eyeShiftScale = new CalibratableScale() {
		public void delta(float f) {
			global.eyeShift += f / 10f;
			Log.e("AR", "eyeShift: " + global.eyeShift);
		}

		public void render(float[] pose) {
			GLES20.glViewport(0, 0, surfaceWidth / 2, surfaceHeight);
			Utils.noGlError();

			Matrix.setIdentityM(shiftMatrix, 0);
			Matrix.translateM(shiftMatrix, 0, -global.eyeShift / 2, 0, 0);
			Matrix.setLookAtM(viewMatrix, 0,
					0f, 0f, 4000f,
					0f, 0f, -5f,
					0f, 1f, 0f);
			Matrix.scaleM(viewMatrix, 0, 2000f, 2000f, 2000f);
			Utils.noGlError();
			drawCube(0f, 0f, -0.4f, 0.015f);
			Utils.noGlError();

			GLES20.glViewport(surfaceWidth / 2, 0, surfaceWidth / 2, surfaceHeight);
			Utils.noGlError();
			Matrix.setIdentityM(shiftMatrix, 0);
			Matrix.translateM(shiftMatrix, 0, global.eyeShift / 2, 0, 0);
			Matrix.setLookAtM(viewMatrix, 0,
					0f, 0f, 4000f,
					0f, 0f, -5f,
					0f, 1f, 0f);
			Matrix.scaleM(viewMatrix, 0, 2000f, 2000f, 2000f);
			Utils.noGlError();
			drawCube(0f, 0f, -0.4f, 0.015f);
			Utils.noGlError();
		}
	};

	private CalibratableScale eyeDistanceScale = new CalibratableScale() {
		public void delta(float f) {
			global.eyeDistance += f / 100f;
			Log.e("AR", "eyeDistance: " + global.eyeDistance);
		}

		public void render(float[] pose) {
			GLES20.glViewport(0, 0, surfaceWidth / 2, surfaceHeight);
			Utils.noGlError();

			Matrix.setIdentityM(shiftMatrix, 0);
			Matrix.translateM(shiftMatrix, 0, -global.eyeShift / 2, 0, 0);
			Matrix.setLookAtM(viewMatrix, 0,
					-global.eyeDistance / 2, 0f, 0f,
					-global.eyeDistance / 2, 0f, -5f,
					0f, 1f, 0f);
			Utils.noGlError();
			drawScene();
			Utils.noGlError();

			GLES20.glViewport(surfaceWidth / 2, 0, surfaceWidth / 2, surfaceHeight);
			Utils.noGlError();
			Matrix.setIdentityM(shiftMatrix, 0);
			Matrix.translateM(shiftMatrix, 0, global.eyeShift / 2, 0, 0);
			Matrix.setLookAtM(viewMatrix, 0,
					global.eyeDistance / 2, 0f, 0f,
					global.eyeDistance / 2, 0f, -5f,
					0f, 1f, 0f);
			Utils.noGlError();
			drawScene();
			Utils.noGlError();
		}
	};

	private CalibratableScale vrScale = new CalibratableScale() {
		public void delta(float f) {
			global.vrScale *= (1 + f / 10f);
			Log.e("AR", "vrScale: " + global.vrScale);

			global.accelerometerScale /= (1 + f / 10f);
			JNI.SVO_setAccelerometerScale(global.accelerometerScale);
		}

		public void render(float[] pose) {
			GLES20.glViewport(0, 0, surfaceWidth / 2, surfaceHeight);
			Utils.noGlError();

			Matrix.setIdentityM(shiftMatrix, 0);
			Matrix.translateM(shiftMatrix, 0, -global.eyeShift / 2, 0, 0);
			Matrix.setLookAtM(lookAtMatrix, 0,
					-global.eyeDistance / 2 - global.cameraDistance, 0f, 0f,
					-global.eyeDistance / 2 - global.cameraDistance, 0f, -5f,
					0f, 1f, 0f);
			Utils.noGlError();
			drawEyeView(pose);
			Utils.noGlError();

			GLES20.glViewport(surfaceWidth / 2, 0, surfaceWidth / 2, surfaceHeight);
			Utils.noGlError();
			Matrix.setIdentityM(shiftMatrix, 0);
			Matrix.translateM(shiftMatrix, 0, global.eyeShift / 2, 0, 0);
			Matrix.setLookAtM(lookAtMatrix, 0,
					global.eyeDistance / 2 - global.cameraDistance, 0f, 0f,
					global.eyeDistance / 2 - global.cameraDistance, 0f, -5f,
					0f, 1f, 0f);
			Utils.noGlError();
			drawEyeView(pose);
			Utils.noGlError();
		}
	};

	private CalibratableScale accelerometerScale = new CalibratableScale() {
		public void delta(float f) {
			global.accelerometerScale += f / 10f;
			Log.e("AR", "accelerometerScale: " + global.accelerometerScale);

			JNI.SVO_setAccelerometerScale(global.accelerometerScale);
		}

		public void render(float[] pose) {
			GLES20.glViewport(0, 0, surfaceWidth / 2, surfaceHeight);
			Utils.noGlError();

			Matrix.setIdentityM(shiftMatrix, 0);
			Matrix.translateM(shiftMatrix, 0, -global.eyeShift / 2, 0, 0);
			Matrix.setLookAtM(lookAtMatrix, 0,
					-global.eyeDistance / 2 - global.cameraDistance, 0f, 0f,
					-global.eyeDistance / 2 - global.cameraDistance, 0f, -5f,
					0f, 1f, 0f);
			Utils.noGlError();
			drawEyeView(pose);
			Utils.noGlError();

			GLES20.glViewport(surfaceWidth / 2, 0, surfaceWidth / 2, surfaceHeight);
			Utils.noGlError();
			Matrix.setIdentityM(shiftMatrix, 0);
			Matrix.translateM(shiftMatrix, 0, global.eyeShift / 2, 0, 0);
			Matrix.setLookAtM(lookAtMatrix, 0,
					global.eyeDistance / 2 - global.cameraDistance, 0f, 0f,
					global.eyeDistance / 2 - global.cameraDistance, 0f, -5f,
					0f, 1f, 0f);
			Utils.noGlError();
			drawEyeView(pose);
			Utils.noGlError();
		}
	};

	public void onTouchEvent(MotionEvent e) {
		menu.onTouchEvent(e);
	}

	public void onKeyEvent(KeyEvent e) { }

	public void onPause() {};
	public void onResume() {};

	public GLSurfaceView.Renderer getRenderer() {
		return this;
	}

	private VideoHistory videoHistory;
	private CameraTracker cameraTracker;

	public ScaleCalibrationActivity(GlobalState global_) {
		global = global_;

		cameraTracker = global.cameraTracker;
		videoHistory = global.videoHistory;

		loadModelData();
		menu = new Menu();
		menu.add(new Menu.Element() {
			{ x = -0.06f; y = 0.07f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, currentScale == eyeShiftScale? Menu.GREEN: Menu.RED);
			}
			public void onClick() {
				currentScale = eyeShiftScale;
			}
		});
		menu.add(new Menu.Element() {
			{ x = -0.02f; y = 0.07f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, currentScale == eyeDistanceScale? Menu.GREEN: Menu.RED);
			}
			public void onClick() {
				currentScale = eyeDistanceScale;
			}
		});
		menu.add(new Menu.Element() {
			{ x = 0.02f; y = 0.07f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, currentScale == vrScale? Menu.GREEN: Menu.RED);
			}
			public void onClick() {
				currentScale = vrScale;
			}
		});
		menu.add(new Menu.Element() {
			{ x = 0.06f; y = 0.07f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, currentScale == accelerometerScale? Menu.GREEN: Menu.RED);
			}
			public void onClick() {
				currentScale = accelerometerScale;
			}
		});

		menu.add(new Menu.Element() {
			{ x = -0.07f; y = 0.02f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, Menu.RED);
			}
			public void onClick() {
				if(currentScale == null) return;
				currentScale.delta(-1f);
			}
		});
		menu.add(new Menu.Element() {
			{ x = 0.07f; y = 0.02f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, Menu.RED);
			}
			public void onClick() {
				if(currentScale == null) return;
				currentScale.delta(1f);
			}
		});
		menu.add(new Menu.Element() {
			{ x = -0.07f; y = -0.02f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, Menu.RED);
			}
			public void onClick() {
				if(currentScale == null) return;
				currentScale.delta(-0.1f);
			}
		});
		menu.add(new Menu.Element() {
			{ x = 0.07f; y = -0.02f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, Menu.RED);
			}
			public void onClick() {
				if(currentScale == null) return;
				currentScale.delta(0.1f);
			}
		});
		menu.add(new Menu.Element() {
			{ x = -0.07f; y = -0.06f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, Menu.RED);
			}
			public void onClick() {
				if(currentScale == null) return;
				currentScale.delta(-0.01f);
			}
		});
		menu.add(new Menu.Element() {
			{ x = 0.07f; y = -0.06f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, Menu.RED);
			}
			public void onClick() {
				if(currentScale == null) return;
				currentScale.delta(0.01f);
			}
		});

		menu.add(new Menu.Element() {
			{ x = 0.00f; y = -0.08f; s = 0.01f; }

			public void draw(Menu.Renderer r) {
				draw(r, Menu.RED);
			}
			public void onClick() {
				global.main.switchTo(new MainMenuActivity(global));
			}
		});
	}

	// FIXME: This should be moved into some global config space
	private static final long DISPLAY_LAG_NS = 90000000;

	private FloatBuffer cubePositions;
	private FloatBuffer cubeTexCoords;
	private int positionHandle;
	private int texCoordsHandle;

	private int sourceDataHandle;

	private float[] shiftMatrix = new float[16];
	private float[] lookAtMatrix = new float[16];
	private float[] viewMatrix = new float[16];
	private float[] modelMatrix = new float[16];
	private float[] projectionMatrix = new float[16];
	private float[] mvpsMatrix = new float[16];
	private float[] mvpMatrix = new float[16];
	private float[] mvMatrix = new float[16];
	private int mvpsMatrixHandle;

	private int linkedShaderHandleSimple;

	private int surfaceWidth;
	private int surfaceHeight;

	private int cameraWidth;
	private int camerHeight;

	private void loadModelData() {
		float[] positions = {
			-1, -1, -1,   1, -1, -1,
			 1, -1, -1,   1,  1, -1,
			 1,  1, -1,  -1,  1, -1,
			-1,  1, -1,  -1, -1, -1,

			-1, -1, -1,  -1, -1,  1,
			 1, -1, -1,   1, -1,  1,
			 1,  1, -1,   1,  1,  1,
			-1,  1, -1,  -1,  1,  1,

			-1, -1,  1,   1, -1,  1,
			 1, -1,  1,   1,  1,  1,
			 1,  1,  1,  -1,  1,  1,
			-1,  1,  1,  -1, -1,  1,
		};

		cubePositions = ByteBuffer.allocateDirect(positions.length * Utils.BYTES_PER_FLOAT)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
		cubePositions.put(positions).position(0);

		final float[] texCoords = {												
			// Front face
			0.0f, 0.0f, 				
			0.0f, 1.0f,
			1.0f, 0.0f,
			0.0f, 1.0f,
			1.0f, 1.0f,
			1.0f, 0.0f,				
			
			// Right face
			0.0f, 0.0f, 				
			0.0f, 1.0f,
			1.0f, 0.0f,
			0.0f, 1.0f,
			1.0f, 1.0f,
			1.0f, 0.0f,	
			
			// Back face
			0.0f, 0.0f, 				
			0.0f, 1.0f,
			1.0f, 0.0f,
			0.0f, 1.0f,
			1.0f, 1.0f,
			1.0f, 0.0f,	
			
			// Left face
			0.0f, 0.0f, 				
			0.0f, 1.0f,
			1.0f, 0.0f,
			0.0f, 1.0f,
			1.0f, 1.0f,
			1.0f, 0.0f,	
			
			// Top face
			0.0f, 0.0f, 				
			0.0f, 1.0f,
			1.0f, 0.0f,
			0.0f, 1.0f,
			1.0f, 1.0f,
			1.0f, 0.0f,	
			
			// Bottom face
			0.0f, 0.0f, 				
			0.0f, 1.0f,
			1.0f, 0.0f,
			0.0f, 1.0f,
			1.0f, 1.0f,
			1.0f, 0.0f
		};

		cubeTexCoords = ByteBuffer.allocateDirect(texCoords.length * Utils.BYTES_PER_FLOAT)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
		cubeTexCoords.put(texCoords).position(0);
	}

	@Override
	public void onSurfaceCreated(GL10 glUnused, EGLConfig config) {
		menu.getRenderer().onSurfaceCreated(glUnused, config);

		GLES20.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		GLES20.glEnable(GLES20.GL_CULL_FACE);
		GLES20.glEnable(GLES20.GL_DEPTH_TEST);

		linkedShaderHandleSimple = Utils.compileShader(getVertexShader(), getFragmentShaderSimple(),
				new String[] {"a_Position", "a_TexCoordinate"});
	}

	@Override
	public void onSurfaceChanged(GL10 glUnused, int _width, int _height) {
		menu.getRenderer().onSurfaceChanged(glUnused, _width, _height);

		// Set the OpenGL viewport to the same size as the surface.
		surfaceWidth = _width;
		surfaceHeight = _height;

		// Create a new perspective projection matrix. The height will stay the same
		// while the width will vary as per aspect ratio.
		final float ratio = (float) surfaceWidth / surfaceHeight;
		final float zoom = 111f;
		final float left = -ratio / zoom;
		final float right = ratio / zoom;
		final float bottom = -1.0f / zoom;
		final float top = 1.0f / zoom;
		final float near = 0.1f;
		final float far = 50000.0f;

		Matrix.frustumM(projectionMatrix, 0, left, right, bottom, top, near, far);
	}

	@Override
	public void onDrawFrame(GL10 glUnused) {
		// FIXME: This is not optimal; it should use a Choreographer to reduce display latency

		long frameStart = System.nanoTime();
		// Position the eye in front of the origin.
		final float eyeX = 0.0f;
		final float eyeY = 0.0f;
		final float eyeZ = 0.0f;

		// We are looking toward the distance
		final float lookX = 0.0f;
		final float lookY = 0.0f;
		final float lookZ = -5.0f;

		// Set our up vector. This is where our head would be pointing were we holding the camera.
		final float upX = 0.0f;
		final float upY = 1.0f;
		final float upZ = 0.0f;

		// Set program handles for cube drawing.
		mvpsMatrixHandle = GLES20.glGetUniformLocation(linkedShaderHandleSimple, "u_MVPSMatrix");
		positionHandle = GLES20.glGetAttribLocation(linkedShaderHandleSimple, "a_Position");
		texCoordsHandle = GLES20.glGetAttribLocation(linkedShaderHandleSimple, "a_TexCoordinate");
		sourceDataHandle = GLES20.glGetAttribLocation(linkedShaderHandleSimple, "u_SourceTexture");

		GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

		Utils.noGlError();
		if(videoHistory.size() == 0) return;

		float[] pose = new float[7];
		cameraTracker.getTransformationAt(System.nanoTime() + DISPLAY_LAG_NS, pose);

		if(currentScale != null) {
			currentScale.render(pose);
		}

		Utils.noGlError();
		long frameEnd = System.nanoTime();
		Log.e("AR", "frame rendering took: " + (frameEnd - frameStart) / 1000 + "us");

		menu.getRenderer().onDrawOverlay();
	}

	private void drawEyeView(float[] pose) {
		final float qi = pose[3];
		final float qj = -pose[4];
		final float qk = -pose[5];
		final float qr = pose[6];
		final float[] rotationMatrix = new float[] {
			1f - 2f*(qj*qj + qk*qk), 2f*(qi*qj - qk*qr), 2f*(qi*qk + qj*qr), 0f,
			2f*(qi*qj + qk*qr), 1f - 2f*(qi*qi + qk*qk), 2f*(qj*qk - qi*qr), 0f,
			2f*(qi*qk - qj*qr), 2f*(qj*qk + qi*qr), 1f - 2f*(qi*qi + qj*qj), 0f,
			0f,                 0f,                 0f,                      1f
		};

		Matrix.multiplyMM(viewMatrix, 0, lookAtMatrix, 0, rotationMatrix, 0);
		Matrix.translateM(viewMatrix, 0,
				global.vrScale * -pose[0],
				global.vrScale * pose[1],
				global.vrScale * pose[2]);

		long start = System.nanoTime();
		drawScene();
		long end = System.nanoTime();
		Log.e("AR", "drawScene took: " + (end - start) / 1000 + "us");
	}

	private void drawScene() {
		drawCube(0f, 0f, -0.4f, 0.015f);

		drawCube(0f, 0f, -0.4f - 0.03f, 0.003f);
		drawCube(0f, 0f, -0.4f + 0.03f, 0.003f);
		drawCube(-0.03f, 0f, -0.4f, 0.003f);
		drawCube(0.03f, 0f, -0.4f, 0.003f);
		drawCube(0f, -0.03f, -0.4f, 0.003f);
		drawCube(0f, 0.03f, -0.4f, 0.003f);

		drawCube(0f, 0f, -0.2f, 0.003f);
		drawCube(0f, 0f, -0.6f, 0.003f);
		drawCube(-0.2f, 0f, -0.4f, 0.003f);
		drawCube(0.2f, 0f, -0.4f, 0.003f);
		drawCube(0f, -0.2f, -0.4f, 0.003f);
		drawCube(0f, 0.2f, -0.4f, 0.003f);
	}
	
	private void drawCube(float x, float y, float z, float scale) {
		Matrix.setIdentityM(modelMatrix, 0);
		Utils.noGlError();

		// Set our per-vertex lighting program.
		GLES20.glUseProgram(linkedShaderHandleSimple);
		Utils.noGlError();

		// Pass in the position information
		cubePositions.position(0);
		Utils.noGlError();
		GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
				0, cubePositions);
		GLES20.glEnableVertexAttribArray(positionHandle);
		Utils.noGlError();

		// Pass in the texture coordinate information
		cubeTexCoords.position(0);
		Utils.noGlError();
		GLES20.glVertexAttribPointer(texCoordsHandle, 2, GLES20.GL_FLOAT, false,
				0, cubeTexCoords);
		GLES20.glEnableVertexAttribArray(texCoordsHandle);
		Utils.noGlError();

		// Model transformations
		Matrix.translateM(modelMatrix, 0, x, y, z);
		Matrix.scaleM(modelMatrix, 0, scale, scale, scale);

		Utils.noGlError();
		float alpha = System.nanoTime() / 1000000000.0f;
		Matrix.rotateM(modelMatrix, 0, alpha * 20, 1f, 1f, 1f);

		Matrix.multiplyMM(mvMatrix, 0, viewMatrix, 0, modelMatrix, 0);
		Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, mvMatrix, 0);
		Matrix.multiplyMM(mvpsMatrix, 0, shiftMatrix, 0, mvpMatrix, 0);
		Utils.noGlError();
		GLES20.glUniformMatrix4fv(mvpsMatrixHandle, 1, false, mvpsMatrix, 0);

		// Actually draw
		// GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, 36);
		GLES20.glDrawArrays(GLES20.GL_LINES, 0, 24);
		Utils.noGlError();
	}

	protected String getVertexShader() {
		// Define our per-pixel lighting shader.
		final String perPixelVertexShader =
				"uniform mat4 u_MVPSMatrix;      \n"		// A constant representing the combined model/view/projection matrix.
			+ "attribute vec4 a_Position;     \n"		// Per-vertex position information we will pass in.
			+ "attribute vec2 a_TexCoordinate; \n"	// Per-vertex texcoords information we will pass in.
			+ "varying vec2 v_TexCoordinate;  \n"		// This will be passed into the fragment shader.

			// The entry point for our vertex shader.
			+ "void main()                                                \n"
			+ "{                                                          \n"
			+ "   v_TexCoordinate = a_TexCoordinate;                      \n" // pass through texture coords
			// gl_Position is a special variable used to store the final position.
			// Multiply the vertex by the matrix to get the final point in normalized screen coordinates.
			+ "   gl_Position = u_MVPSMatrix * a_Position;                 \n"
			+ "}                                                          \n";

		return perPixelVertexShader;
	}

	protected String getFragmentShader() {
		final String perPixelFragmentShader =
				"precision mediump float;\n" // Set the default precision to medium.
			+ "uniform sampler2D u_SourceTexture;\n"
			+ "varying vec2 v_TexCoordinate;\n"
			+ "void main() {\n"
			+ "   gl_FragColor = vec4(0.0, 0.0, 0.0, 0.0) + texture2D(u_SourceTexture, v_TexCoordinate); \n"
			+ "}\n";

		return perPixelFragmentShader;
	}

	protected String getFragmentShaderSimple() {
		final String perPixelFragmentShader =
				"precision mediump float;\n" // Set the default precision to medium.
			+ "varying vec2 v_TexCoordinate;\n"
			+ "void main() {\n"
			// + "   gl_FragColor = vec4(v_TexCoordinate.x, v_TexCoordinate.y, 1.0, 0.0); \n"
			// include tex coords here so attribute is not optimized out (and -1 returned from getAttributeHandle)
			+ "   gl_FragColor = vec4(0, 1, 0, 0.0) + v_TexCoordinate; \n"
			+ "}\n";

		return perPixelFragmentShader;
	}
}
