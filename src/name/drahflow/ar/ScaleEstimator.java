package name.drahflow.ar;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import java.util.ArrayList;
import android.util.Log;
import android.os.Handler;
import android.os.HandlerThread;

class ScaleEstimator {
	private SensorManager sensorManager;
	private Sensor accelerometer;
	private SensorEventListener accelerometerListener;
	private Sensor gyroscope;
	private SensorEventListener gyroscopeListener;
	private VideoHistory video;
	private CameraTracker tracker;

	private HandlerThread sensorThread;
	private Handler sensorHandler;

	private ArrayList<float[]> sensorHistory = new ArrayList<>();
	private float xScale, yScale, zScale;

	public ScaleEstimator(SensorManager _sensorManager, VideoHistory _video, CameraTracker _tracker) {
		video = _video;
		tracker = _tracker;

		xScale = yScale = zScale = 1.0f;

		sensorThread = new HandlerThread("Sensor Processing");
		sensorThread.start();
		sensorHandler = new Handler(sensorThread.getLooper());

		sensorManager = _sensorManager;
		accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		accelerometerListener = new SensorEventListener() {
			@Override
			public void onAccuracyChanged(Sensor s, int accuracy) {}

			@Override
			public void onSensorChanged(SensorEvent e) {
				Log.e("AR", String.format("Acceleration: %6.4f,%6.4f,%6.4f", e.values[0], e.values[1], e.values[2]));
				tracker.processAccelerometerEvent(e);
			}
		};

		gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		gyroscopeListener = new SensorEventListener() {
			@Override
			public void onAccuracyChanged(Sensor s, int accuracy) {}

			@Override
			public void onSensorChanged(SensorEvent e) {
				Log.e("AR", String.format("Gyroscope: %6.4f,%6.4f,%6.4f", e.values[0], e.values[1], e.values[2]));
				tracker.processGyroscopeEvent(e);
			}
		};
	}

	public void onResume() {
		sensorManager.registerListener(accelerometerListener, accelerometer, SensorManager.SENSOR_DELAY_NORMAL, sensorHandler);
		sensorManager.registerListener(gyroscopeListener, gyroscope, SensorManager.SENSOR_DELAY_GAME, sensorHandler);
	}
	
	public void onPause() {
		sensorManager.unregisterListener(accelerometerListener);
		sensorManager.unregisterListener(gyroscopeListener);
	}
	
	private final static float EPSILON = 1e-6f;
	
	private void calibrateAccelerometer() {
		// xScale = yScale = zScale = 1.0f;
		// \sum_i sqrt(A * h_i.x^2 + B * h_i.y^2 + C * h_i.z^2) = i * 9.81m^2
		final float stepSize = 0.01f / sensorHistory.size();

		Log.e("AR", "Running calibration");
		// TODO: put real optimization algorithm in here
		final float expectedTotal = sensorHistory.size() * 9.805f;
		for(int i = 0; i < 100; ++i) {
			float total = calculateTotalAcceleration(xScale, yScale, zScale);
			float total_dx = calculateTotalAcceleration(xScale + EPSILON, yScale, zScale) - 
				calculateTotalAcceleration(xScale - EPSILON, yScale, zScale);
			float total_dy = calculateTotalAcceleration(xScale, yScale + EPSILON, zScale) -
				calculateTotalAcceleration(xScale, yScale - EPSILON, zScale);
			float total_dz = calculateTotalAcceleration(xScale, yScale, zScale + EPSILON) -
				calculateTotalAcceleration(xScale, yScale, zScale - EPSILON);

			float error = expectedTotal - total;
			Log.e("AR", String.format("calibration error: %8.6f", error));

			float length = (float)(Math.sqrt(total_dx * total_dx + total_dy * total_dy + total_dz * total_dz));
			float dist = error / length;

			xScale += stepSize * dist * total_dx;
			yScale += stepSize * dist * total_dy;
			zScale += stepSize * dist * total_dz;

			Log.e("AR", String.format("calibration %d: %8.6f,%8.6f,%8.6f", i, xScale, yScale, zScale));
		}

		sensorHistory.clear();
	}

	private float calculateTotalAcceleration(float sx, float sy, float sz) {
		float sum = 0.0f;
		sx *= sx;
		sy *= sy;
		sz *= sz;

		for(float[] a: sensorHistory) {
			sum += (float)(Math.sqrt(sx * a[0] * a[0] + sy * a[1] * a[1] + sz * a[2] * a[2]));
		}

		return sum;
	}
}
