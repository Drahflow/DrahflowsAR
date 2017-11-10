package name.drahflow.ar;

import android.opengl.Matrix;
import android.util.Log;
import android.hardware.SensorEvent;
import android.os.Looper;
import android.os.Handler;
import android.os.HandlerThread;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.CaptureFailure;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.params.RggbChannelVector;
import java.util.Arrays;
import android.media.Image;
import android.media.ImageReader;
import android.graphics.ImageFormat;
import android.content.Context;
import android.view.Surface;

public class CameraTracker {
  private int width;
  private int height;
	private VideoHistory history;

	private HandlerThread cameraThread;
	private Handler cameraHandler;
	private CameraManager cameraManager;
	private CameraDevice camera;
	private ImageReader cameraReader;

	public enum CameraState {
		CAMERA_NOT_AVAILABLE,
		CAMERA_BOOTING,
		CAMERA_OPERATIONAL;
	};

	private CameraState cameraState;
	public float[] debugImage;

  public CameraTracker(CameraManager _cameraManager, int _width, int _height, final VideoHistory _history) {
		cameraManager = _cameraManager;
    width = _width;
    height = _height;
		history = _history;

		debugImage = new float[width * height];

		cameraState = CameraState.CAMERA_NOT_AVAILABLE;

		cameraThread = new HandlerThread("Camera Processing");
		cameraThread.start();
		cameraHandler = new Handler(cameraThread.getLooper());

		cameraReader = ImageReader.newInstance(width, height, ImageFormat.YUV_420_888, 4);
		cameraReader.setOnImageAvailableListener(new ImageReader.OnImageAvailableListener() {
			@Override
			public void onImageAvailable(ImageReader cameraReader) {
				Image frame = cameraReader.acquireLatestImage();
				if(frame == null) return;

				history.addFrame(frame);
				frame.close();

				Log.e("AR", "Camera state: " + cameraState);
				if(cameraState != CameraState.CAMERA_OPERATIONAL) {
					// camera not initialized fully, yet
					if(cameraState == CameraState.CAMERA_BOOTING && history.size() > 10) {
						history.clear();
						cameraState = CameraState.CAMERA_OPERATIONAL;
					}
					return;
				} else if(history.size() < 3) {
					// algorithms need 3 frames at least
					return;
				}

				processFrame();
			}
		}, cameraHandler);

		initialize();
	}

	public void initialize() {
		// TODO: drop (or forward) unused calibration parameters
		JNI.SVO_prepare(width, height, 0.0f, 0.0f, 0.0, 0.0);
	}

	private void startCamera() {
		cameraState = CameraState.CAMERA_NOT_AVAILABLE;

		try {
			String[] cameras = cameraManager.getCameraIdList();

			for(int i = 0; i < cameras.length; ++i) {
				Log.e("AR", "Camera: " + cameras[i]);
			}

			cameraManager.openCamera(cameras[0], new CameraDevice.StateCallback() {
				public void onOpened(CameraDevice c) {
					camera = c;

					final Surface surface = cameraReader.getSurface();

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
													if(cameraState == CameraState.CAMERA_NOT_AVAILABLE) {
														cameraState = CameraState.CAMERA_BOOTING;
													}
												}
											}, cameraHandler);
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
			}, cameraHandler);
		} catch(CameraAccessException cae) {
			throw new Error("May not open camera", cae);
		}
	}

	protected void stopCamera() {
		if(camera != null) {
			camera.close();
		}
	}

	public void onPause() { stopCamera(); }
	public void onResume() { startCamera(); }

	public boolean hasGoodTracking() {
		return JNI.SVO_hasGoodTracking();
	}

  public void processFrame() {
		VideoFrame lastFrame = history.getLastFrame();

		// Log.e("AR", "Camera processing on thread: " + Thread.currentThread().getName());
		JNI.SVO_processFrame(lastFrame.getIntensities(), lastFrame.getTimestamp());

		float[] transformation = new float[7];
		JNI.SVO_getTransformation(lastFrame.getTimestamp(), transformation);
		lastFrame.setTransformation(transformation);

		long start = System.nanoTime();
		JNI.Gesture_processFrame(debugImage);
		long end = System.nanoTime();
		// Log.e("AR", "gesture processing took: " + (float)(end - start) / 1000000 + " ms");
  }

	public void processAccelerometerEvent(SensorEvent e) {
		// Log.e("AR", "Accelerometer processing on thread: " + Thread.currentThread().getName());
		JNI.SVO_processAccelerometer(e.values, e.timestamp);
	}

	public void processGyroscopeEvent(SensorEvent e) {
		// Log.e("AR", "Gyroscope processing on thread: " + Thread.currentThread().getName());
		JNI.SVO_processGyroscope(e.values, e.timestamp);
	}

	public void getTransformationAt(long time_nano, float[] transformation) {
		// Log.e("AR", "Pose estimation on thread: " + Thread.currentThread().getName());
		JNI.SVO_getTransformation(time_nano, transformation);

		//Log.e("AR", "pose: " +
		//		String.format("%8.6f,%8.6f,%8.6f @ %8.6f,%8.6f,%8.6f,%8.6f",
		//			transformation[0], transformation[1], transformation[2],
		//			transformation[3], transformation[4], transformation[5], transformation[6]));
	}
}
