package name.drahflow.ar;

public class VideoFrame {
	public long timestamp;
	public float[] intensities;
	public float[] transformation;

	public VideoFrame(int width, int height, long _timestampe, float[] _intensities) {
		timestamp = _timestampe;
		intensities = _intensities;
	}

	public float[] getIntensities() {
		return intensities;
	}

	public void setTransformation(float[] _transformation) {
		transformation = _transformation;
	}

	public float[] getTransformation() {
		return transformation;
	}
}
