package name.drahflow.ar;

import android.app.Activity;
import android.app.ActivityManager;
import android.content.Context;
import android.content.pm.ConfigurationInfo;
import android.os.Bundle;
import android.os.Looper;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Environment;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.ArrayList;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;
import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.GLSurfaceView;
import android.opengl.GLES20;
import android.opengl.GLES30;
import android.opengl.GLES11Ext;
import android.opengl.Matrix;
import android.opengl.GLUtils;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.Log;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.SurfaceTexture;
import android.view.View;
import android.view.Surface;
import android.view.Window;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.CaptureFailure;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.params.RggbChannelVector;
import com.epson.moverio.btcontrol.DisplayControl;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.FileOutputStream;

public class DrahflowsAR extends Activity {
	private GLSurfaceView mainView;
	private DevelopmentRenderer mainRenderer;
	private VideoHistory videoHistory;
	private CameraTracker cameraTracker;

	private static final int CAMERA_NOT_AVAILABLE = 0;
	private static final int CAMERA_BOOTING = 1;
	private static final int CAMERA_OPERATIONAL = 2;
	private int cameraState;

	private static final int camera_width = 640;
	private static final int camera_height = 480;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

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
		cameraState = CAMERA_NOT_AVAILABLE;

		videoHistory = new VideoHistory(width, height, camera_width, camera_height, 30l * 1000l * 1000l * 1000l);
		cameraTracker = new CameraTracker(width, height, videoHistory);
		mainRenderer = new DevelopmentRenderer(width, height);
		mainView = new GLSurfaceView(this);
		mainView.setEGLContextClientVersion(2);
		mainView.setRenderer(mainRenderer);
		setContentView(mainView);
	}

	private CameraDevice camera;
	private HandlerThread mBackgroundThread;
	private Handler mBackgroundHandler;
	private SurfaceTexture cameraTexture;
	private int cameraTextureHandle;

	@Override
	protected void onResume() {
		// Ideally a game should implement onResume() and onPause()
		// to take appropriate action when the activity looses focus
		super.onResume();
		mainView.onResume();
		// new DisplayControl(this).setMode(DisplayControl. DISPLAY_MODE_3D, false);

		startCamera();
	}

	protected void startCamera() {
		// not yet possible
		if(cameraTexture == null) return;
		cameraState = CAMERA_NOT_AVAILABLE;

		// mBackgroundThread = new HandlerThread("Camera Processing");
		// mBackgroundThread.start();
		// mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
		mBackgroundHandler = new Handler(); // FIXME

		CameraManager cameraManager = (CameraManager)getSystemService(Context.CAMERA_SERVICE);
		try {
			String[] cameras = cameraManager.getCameraIdList();

			for(int i = 0; i < cameras.length; ++i) {
				Log.e("AR", "Camera: " + cameras[i]);
			}

			cameraManager.openCamera(cameras[0], new CameraDevice.StateCallback() {
				public void onOpened(CameraDevice c) {
					camera = c;

					// cameraTexture.setDefaultBufferSize(1920, 1080);
					cameraTexture.setDefaultBufferSize(camera_width, camera_height);
					final Surface surface = new Surface(cameraTexture);

					try {
						c.createCaptureSession(Arrays.asList(surface),
								new CameraCaptureSession.StateCallback() {
									@Override
									public void onConfigured(CameraCaptureSession cameraCaptureSession) {
										// The camera is already closed
										if (camera == null) {
											return;
										}

										try {
											// Finally, we start displaying the camera preview.
											CaptureRequest.Builder captureRequestBuilder = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
											captureRequestBuilder.addTarget(surface);
											captureRequestBuilder.set(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO);
											captureRequestBuilder.set(CaptureRequest.COLOR_CORRECTION_MODE, CaptureRequest.COLOR_CORRECTION_MODE_TRANSFORM_MATRIX);
											captureRequestBuilder.set(CaptureRequest.COLOR_CORRECTION_GAINS, new RggbChannelVector(20.0f, 5.0f, 5.0f, 5.0f));
											captureRequestBuilder.set(CaptureRequest.SENSOR_EXPOSURE_TIME, 20000000l);  // in usecs
											captureRequestBuilder.set(CaptureRequest.SENSOR_SENSITIVITY, 50000);
											CaptureRequest captureRequest = captureRequestBuilder.build();

											cameraCaptureSession.setRepeatingRequest(captureRequest, new CameraCaptureSession.CaptureCallback() {
												public void onCaptureBufferLost(CameraCaptureSession session, CaptureRequest request, Surface target, long frameNumber) { }
												public void onCaptureCompleted(CameraCaptureSession session, CaptureRequest request, TotalCaptureResult result) { }
												public void onCaptureFailed(CameraCaptureSession session, CaptureRequest request, CaptureFailure failure) { }
												public void onCaptureProgressed(CameraCaptureSession session, CaptureRequest request, CaptureResult partialResult) { }
												public void onCaptureSequenceAborted(CameraCaptureSession session, int sequenceId) { }
												public void onCaptureSequenceCompleted(CameraCaptureSession session, int sequenceId, long frameNumber) { }
												public void onCaptureStarted(CameraCaptureSession session, CaptureRequest request, long timestamp, long frameNumber) {
													if(cameraState == CAMERA_NOT_AVAILABLE) {
														cameraState = CAMERA_BOOTING;
													}
												}
											}, mBackgroundHandler);
										} catch (CameraAccessException e) {
											e.printStackTrace();
										}
									}

									@Override
									public void onConfigureFailed(CameraCaptureSession cameraCaptureSession) {
										Log.e("AR", "Capture Config failed");
									}
								},
								null);
					} catch (CameraAccessException e) {
						e.printStackTrace();
					}
				}

				public void onError(CameraDevice c, int error) {
					c.close();
					camera = null;
				}
				public void onDisconnected(CameraDevice c) {
					c.close();
					camera = null;
				}
				public void onClosed(CameraDevice c) {
					camera = null;
				}
			}, mBackgroundHandler);
		} catch(CameraAccessException cae) {
			throw new Error("May not open camera", cae);
		}
	}

	@Override
	protected void onPause() {
		// Ideally a game should implement onResume() and onPause()
		// to take appropriate action when the activity looses focus
		super.onPause();
		mainView.onPause();
		new DisplayControl(this).setMode(DisplayControl. DISPLAY_MODE_2D, false);

		stopCamera();
	}

	protected void stopCamera() {
		if(camera != null) {
			camera.close();
		}

		if(mBackgroundThread != null) {
			mBackgroundThread.getLooper().quitSafely();
			mBackgroundHandler = null;
		}
	}

	public class DevelopmentRenderer implements GLSurfaceView.Renderer {
		private FloatBuffer cubePositions;
		private FloatBuffer cubeTexCoords;
		private int positionHandle;
		private int texCoordsHandle;

		private int sourceDataHandle;

		private float[] lookAtMatrix = new float[16];
		private float[] projectionMatrix = new float[16];
		private float[] mvpMatrix = new float[16];
		private int mvpMatrixHandle;

		private int linkedShaderHandle;

		private int width;
		private int height;

		private int dataWidth;
		private int dataHeight;

		public DevelopmentRenderer(int _dataWidth, int _dataHeight) {
			loadModelData();

			dataWidth = _dataWidth;
			dataHeight = _dataHeight;
		}

		private void loadModelData() {
			float[] positions = {
				-0.05f, 0.025f, 0.0f,
				-0.05f, -0.025f, 0.0f,
				0.05f, 0.025f, 0.0f,
				-0.05f, -0.025f, 0.0f,
				0.05f, -0.025f, 0.0f,
				0.05f, 0.025f, 0.0f,
			};

			cubePositions = ByteBuffer.allocateDirect(positions.length * Utils.BYTES_PER_FLOAT)
					.order(ByteOrder.nativeOrder()).asFloatBuffer();
			cubePositions.put(positions).position(0);

			float[] texCoords = {
				0.0f, 0.0f,
				0.0f, 1.0f,
				1.0f, 0.0f,
				0.0f, 1.0f,
				1.0f, 1.0f,
				1.0f, 0.0f,
			};

			cubeTexCoords = ByteBuffer.allocateDirect(texCoords.length * Utils.BYTES_PER_FLOAT)
					.order(ByteOrder.nativeOrder()).asFloatBuffer();
			cubeTexCoords.put(texCoords).position(0);
		}

		@Override
		public void onSurfaceCreated(GL10 glUnused, EGLConfig config) {
			int[] someTexs = new int[1];
			GLES20.glGenTextures(1, someTexs, 0);
			cameraTextureHandle = someTexs[0];

			GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, cameraTextureHandle);
			GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE);
			GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE);
			GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
			GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
			cameraTexture = new SurfaceTexture(cameraTextureHandle);

			new Handler(Looper.getMainLooper()).post(new Runnable() {
				public void run() { startCamera(); }
			});

			GLES20.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			GLES20.glEnable(GLES20.GL_CULL_FACE);
			GLES20.glEnable(GLES20.GL_DEPTH_TEST);

			linkedShaderHandle = Utils.compileShader(getVertexShader(), getFragmentShader(),
					new String[] {"a_Position", "a_TexCoordinate"});
		}

		@Override
		public void onSurfaceChanged(GL10 glUnused, int _width, int _height) {
			// Set the OpenGL viewport to the same size as the surface.
			width = _width;
			height = _height;

			// Create a new perspective projection matrix. The height will stay the same
			// while the width will vary as per aspect ratio.
			final float ratio = (float) width / height;
			final float zoom = 110f;
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
			// Position the eye in front of the origin.
			final float eyeX = 0.0f;
			final float eyeY = 0.0f;
			final float eyeZ = 0.50f;

			// We are looking toward the distance
			final float lookX = 0.0f;
			final float lookY = 0.0f;
			final float lookZ = -5.0f;

			// Set our up vector. This is where our head would be pointing were we holding the camera.
			final float upX = 0.0f;
			final float upY = 1.0f;
			final float upZ = 0.0f;

			// Set program handles for cube drawing.
			mvpMatrixHandle = GLES20.glGetUniformLocation(linkedShaderHandle, "u_MVPMatrix");
			positionHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_Position");
			texCoordsHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_TexCoordinate");
			sourceDataHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "u_SourceTexture");

			GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);
			cameraTexture.updateTexImage();

			videoHistory.addFrame(cameraTexture, cameraTextureHandle);
			Utils.noGlError();

			Log.e("AR", "Camera state: " + cameraState);
			if(cameraState != CAMERA_OPERATIONAL) {
				// camera not initialized fully, yet
				if(cameraState == CAMERA_BOOTING && videoHistory.size() > 10) {
					videoHistory.clear();
					cameraState = CAMERA_OPERATIONAL;
				}
				return;
			} else if(videoHistory.size() < 3) {
				// algorithms need 3 frames at least
				return;
			}

			cameraTracker.processFrame();

			long start = System.nanoTime();
			Log.e("AR", "history size: " + videoHistory.size());

			long end = System.nanoTime();
			Log.e("AR", "depth updating etc. took: " + (float)(end - start) / 1000000 + " ms");

			Utils.noGlError();

			// Set the view matrix. This matrix can be said to represent the camera position.
			// NOTE: In OpenGL 1, a ModelView matrix is used, which is a combination of a model and
			// view matrix. In OpenGL 2, we can keep track of these matrices separately if we choose.
			GLES20.glViewport(0, 0, width / 2, height);
			Matrix.setLookAtM(lookAtMatrix, 0, eyeX - 0.03f, eyeY, eyeZ, lookX, lookY, lookZ, upX, upY, upZ);
			// FIXME: Keyframingi
			drawDebugImage(videoHistory.getLastFrame().getIntensities());
			Utils.noGlError();

			GLES20.glViewport(width / 2, 0, width / 2, height);
			Matrix.setLookAtM(lookAtMatrix, 0, eyeX + 0.03f, eyeY, eyeZ, lookX, lookY, lookZ, upX, upY, upZ);
			// drawDebugImage(depthEstimate.debugProjectionBuffer, depthEstimate.getDepthEstimate());

			Utils.noGlError();
		}

		private void drawDebugImage(int sourceTextureHandle) {
			// Set our per-vertex lighting program.
			GLES20.glUseProgram(linkedShaderHandle);

			// Pass in the position information
			cubePositions.position(0);
			GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
					0, cubePositions);
			GLES20.glEnableVertexAttribArray(positionHandle);

			// Pass in the texture coordinate information
			cubeTexCoords.position(0);
			GLES20.glVertexAttribPointer(texCoordsHandle, 2, GLES20.GL_FLOAT, false,
					0, cubeTexCoords);
			GLES20.glEnableVertexAttribArray(texCoordsHandle);

			GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
			GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, sourceTextureHandle);
			GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_NEAREST);
			GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_NEAREST);
			GLES20.glUniform1i(sourceDataHandle, 0);

			Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, lookAtMatrix, 0);
			GLES20.glUniformMatrix4fv(mvpMatrixHandle, 1, false, mvpMatrix, 0);

			// Actually draw
			GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, 6);
		}

		private void drawDebugImage(float[] sourceIntensities) {
			drawDebugImage(dataWidth, dataHeight, sourceIntensities, null, null);
		}

		private void drawDebugImage(float[] sourceIntensities, float[] sourceDepths) {
			drawDebugImage(dataWidth, dataHeight, sourceIntensities, sourceDepths, null);
		}

		private void drawDebugImage(float[] sourceIntensities, float[] sourceDepths, float[] sourceVariances) {
			drawDebugImage(dataWidth, dataHeight, sourceIntensities, sourceDepths, sourceVariances);
		}

		private void drawDebugImage(int debugWidth, int debugHeight, float[] sourceIntensities) {
			drawDebugImage(debugWidth, debugHeight, sourceIntensities, null, null);
		}

		private FloatBuffer tmpBuffer;
		private void drawDebugImage(int debugWidth, int debugHeight, float[] sourceIntensities, float[] sourceDepths, float[] sourceVariances) {
			int[] someTexs = new int[1];
			GLES20.glGenTextures(1, someTexs, 0);
			int tmpTex = someTexs[0];

			if(tmpBuffer == null) {
				tmpBuffer = ByteBuffer.allocateDirect(4 * debugWidth * debugHeight * Utils.BYTES_PER_FLOAT)
						.order(ByteOrder.nativeOrder()).asFloatBuffer();
			}

			for(int i = 0; i < debugWidth * debugHeight; ++i) {
				tmpBuffer.put(i * 4, 0.5f);
				if(sourceIntensities != null) {
					tmpBuffer.put(i * 4 + 1, 0.33333f * sourceIntensities[i]);
				}
				if(sourceDepths != null) {
					if(sourceVariances != null) {
						if(sourceVariances[i] < 50.0) {
							tmpBuffer.put(i * 4 + 1, (sourceDepths[i] - 0.3f) * 100.0f / 256.0f);
							tmpBuffer.put(i * 4 + 2, sourceDepths[i] * 10.0f);
						}
					} else {
						tmpBuffer.put(i * 4 + 1, (sourceDepths[i] - 0.3f) * 100.0f / 256.0f);
						tmpBuffer.put(i * 4 + 2, sourceDepths[i] * 10.0f);
					}
				}
			}

			tmpBuffer.position(0);
			GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, tmpTex);
			GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_RGBA, debugWidth, debugHeight, 0, GLES30.GL_RGBA, GLES30.GL_FLOAT, tmpBuffer);

			drawDebugImage(tmpTex);

			GLES20.glDeleteTextures(1, someTexs, 0);
		}

		protected String getVertexShader() {
			// Define our per-pixel lighting shader.
			final String perPixelVertexShader =
				  "uniform mat4 u_MVPMatrix;      \n"		// A constant representing the combined model/view/projection matrix.
				+ "attribute vec4 a_Position;     \n"		// Per-vertex position information we will pass in.
				+ "attribute vec2 a_TexCoordinate; \n"	// Per-vertex texcoords information we will pass in.
				+ "varying vec2 v_TexCoordinate;  \n"		// This will be passed into the fragment shader.

				// The entry point for our vertex shader.
				+ "void main()                                                \n"
				+ "{                                                          \n"
				+ "   v_TexCoordinate = a_TexCoordinate;                      \n" // pass through texture coords
				// gl_Position is a special variable used to store the final position.
				// Multiply the vertex by the matrix to get the final point in normalized screen coordinates.
				+ "   gl_Position = u_MVPMatrix * a_Position;                 \n"
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
	}
}
