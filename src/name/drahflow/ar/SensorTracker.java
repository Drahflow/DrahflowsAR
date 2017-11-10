package name.drahflow.ar;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import java.util.ArrayList;
import android.util.Log;
import android.os.Handler;
import android.os.HandlerThread;

class SensorTracker {
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

	public SensorTracker(SensorManager _sensorManager, VideoHistory _video, CameraTracker _tracker) {
		video = _video;
		tracker = _tracker;

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
				// Log.e("AR", String.format("Acceleration: %6.4f,%6.4f,%6.4f", e.values[0], e.values[1], e.values[2]));
				tracker.processAccelerometerEvent(e);
			}
		};

		gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		gyroscopeListener = new SensorEventListener() {
			@Override
			public void onAccuracyChanged(Sensor s, int accuracy) {}

			@Override
			public void onSensorChanged(SensorEvent e) {
				// Log.e("AR", String.format("Gyroscope: %6.4f,%6.4f,%6.4f", e.values[0], e.values[1], e.values[2]));
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
}
