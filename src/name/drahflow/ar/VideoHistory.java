package name.drahflow.ar;

import android.graphics.SurfaceTexture;
import android.opengl.GLES20;
import android.opengl.GLES30;
import android.opengl.GLES11Ext;
import java.util.ArrayList;
import android.graphics.SurfaceTexture;
import android.util.Log;
import java.nio.FloatBuffer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import android.media.Image;

public class VideoHistory {
	private ArrayList<VideoFrame> frames = new ArrayList<>();
	private long duration;

	public final int width;
	public final int height;
	private int camera_width;
	private int camera_height;

	private int linkedShaderHandle = -1;
	private int bufferTextureHandle;
	private FloatBuffer outputBuffer;
	private int positionHandle, texCoordsHandle, cameraDataHandle;

	public VideoHistory(int _width, int _height, int _camera_width, int _camera_height, long _duration) {
		duration = _duration;

		width = _width;
		height = _height;
		camera_width = _camera_width;
		camera_height = _camera_height;

		outputBuffer = ByteBuffer.allocateDirect(4 * width * height * Utils.BYTES_PER_FLOAT)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
	}

	public VideoFrame getLastFrame() {
		synchronized(frames) {
			return frames.get(frames.size() - 1);
		}
	}

	public VideoFrame get(int i) {
		return frames.get(i);
	}

	public int size() {
		return frames.size();
	}

	public void clear() {
		frames.clear();
	}

	public void addFrame(SurfaceTexture frame, int frameTextureHandle) {
		cleanup();

		long timestamp = frame.getTimestamp();

		if(linkedShaderHandle < 0) {
			String fragmentShader = cameraCopyFragmentShader();
			if(width == camera_width && height == camera_height) {
				fragmentShader = cameraCopyFragmentShaderUnscaled();
			}

			linkedShaderHandle = Utils.compileShader(Utils.computeVertexShader(), fragmentShader,
					new String[] {"a_Position", "a_TexCoordinate"});

			positionHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_Position");
			texCoordsHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_TexCoordinate");
			cameraDataHandle = GLES20.glGetUniformLocation(linkedShaderHandle, "u_CameraData");

			int[] someTexs = new int[1];
			GLES20.glGenTextures(1, someTexs, 0);
			bufferTextureHandle = someTexs[0];

			GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, bufferTextureHandle);
			GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_R16F, width, height, 0, GLES30.GL_RED, GLES30.GL_FLOAT, null);
			GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_NEAREST);
			GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_NEAREST);
			Utils.noGlError();
		}

		GLES20.glViewport(0, 0, width, height);
		Utils.noGlError();

		// So we can glUniform1i
		GLES20.glUseProgram(linkedShaderHandle);
		GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
		GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, frameTextureHandle);
		// TODO: Try to explicitely downscale in the shader instead
		GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_NEAREST);
		GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_NEAREST);
		GLES20.glUniform1i(cameraDataHandle, 0);

		Utils.bindComputeFramebuffer(bufferTextureHandle);

		GLES20.glViewport(0, 0, width, height);
		GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

		Utils.runShaderComputationRaw(linkedShaderHandle, positionHandle, texCoordsHandle);
		Utils.noGlError();

		outputBuffer.position(0);
		GLES30.glReadPixels(0, 0, width, height, GLES30.GL_RGBA, GLES30.GL_FLOAT, outputBuffer);
		Utils.noGlError();
		float[] intensities = new float[width * height];
		for(int i = 0; i < width * height; ++i) {
			intensities[i] = outputBuffer.get(i * 4);
		}

		GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, 0);

		if(!frames.isEmpty()) {
			frames.get(frames.size() - 1).clearIntensities();
		}

		frames.add(new VideoFrame(width, height, timestamp, intensities));

		Log.e("AR", "Video history size: " + frames.size());
	}

	public void addFrame(Image img) {
		cleanup();

		long timestamp = img.getTimestamp();

		float[] intensities = new float[width * height];
		final ByteBuffer Y = img.getPlanes()[0].getBuffer();
		for(int i = 0; i < width * height; ++i) {
			// Pixel stride: 1, Row stride: 640 on EPSON MOVERIO 300
			intensities[i] = (((short)Y.get(i) + 256) % 256) / 255.0f;
		}

		frames.add(new VideoFrame(width, height, timestamp, intensities));

		Log.e("AR", "Video history size: " + frames.size());
	}

	private String cameraCopyFragmentShader() {
		return
		   	"#extension GL_OES_EGL_image_external : require\n"
			+ "precision mediump float;\n"
			+ "uniform lowp samplerExternalOES u_CameraData;\n"
			+ "varying vec2 v_TexCoordinate;\n"
			+ "void main() {\n"
			+ "   mediump float result = 0.0;\n"
			+ "   vec2 low = v_TexCoordinate;\n"
			+ "   vec2 high = low + vec2(1.0 / " + width + ".0, 1.0 / " + height + ".0);\n"
			+ "   for(mediump float y = low.y; y < high.y; y += " + (1.0 / camera_height) + ") {\n"
			+ "     for(mediump float x = low.x; x < high.x; x += " + (1.0 / camera_width) + ") {\n"
			+ "       lowp vec4 texel = texture2D(u_CameraData, vec2(x, y));\n"
			+ "       result += texel.r + texel.g + texel.b;\n"
			+ "     }\n"
			+ "   }\n"
			+ "   gl_FragColor.r = result / " + 3.0f * ((float)(camera_width / width) * (camera_height / height)) + "f;\n"
			+ "}\n";
	}

	private String cameraCopyFragmentShaderUnscaled() {
		return
		    "#extension GL_OES_EGL_image_external : require\n"
			+ "precision mediump float;\n"
			+ "uniform lowp samplerExternalOES u_CameraData;\n"
			+ "varying vec2 v_TexCoordinate;\n"
			+ "void main() {\n"
			+ "   lowp vec4 texel = texture2D(u_CameraData, v_TexCoordinate);\n"
			+ "   mediump float result = texel.r + texel.g + texel.b;\n"
			+ "   gl_FragColor.r = result / 3.0f;\n"
			+ "}\n";
	}

	private void cleanup() {
		if(frames.size() > 100) frames.remove(0);
		if(frames.size() > 5) frames.get(frames.size() - 5).clearIntensities();
	}
}
