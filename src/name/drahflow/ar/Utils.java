package name.drahflow.ar;

import android.opengl.GLES20;
import android.opengl.GLUtils;
import android.util.Log;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.Canvas;
import android.graphics.Bitmap;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.HashMap;

abstract public class Utils {
	public static final int BYTES_PER_FLOAT = 4;
	public static final int BYTES_PER_SHORT = 2;

	static private int compileShader(final int type, final String source) {
		int handle = GLES20.glCreateShader(type);
		if(handle == 0) throw new RuntimeException("Could not allocate shader.");

		GLES20.glShaderSource(handle, source);
		GLES20.glCompileShader(handle);

		final int[] compileStatus = new int[1];
		GLES20.glGetShaderiv(handle, GLES20.GL_COMPILE_STATUS, compileStatus, 0);

		if(compileStatus[0] == 0) {
			Log.e("AR", "Error compiling shader: " + GLES20.glGetShaderInfoLog(handle));
			GLES20.glDeleteShader(handle);

			throw new RuntimeException("Shader failed to compile.");
		}

		return handle;
	}

	private static HashMap<String, Integer> linkedShaders = new HashMap<>();
	static public int compileShader(final String vertexShader, final String fragmentShader, final String[] attributes) {
		StringBuilder key = new StringBuilder();
		key.append(vertexShader);
		key.append(fragmentShader);
		for(String a: attributes) key.append(a);
		Integer linkedProgram = linkedShaders.get(key.toString());
		if(linkedProgram != null) return linkedProgram;

		final int vertexShaderHandle = compileShader(GLES20.GL_VERTEX_SHADER, vertexShader);
		final int fragmentShaderHandle = compileShader(GLES20.GL_FRAGMENT_SHADER, fragmentShader);

		int programHandle = GLES20.glCreateProgram();

		if(programHandle == 0) throw new RuntimeException("Could not allocate program.");

		GLES20.glAttachShader(programHandle, vertexShaderHandle);
		GLES20.glAttachShader(programHandle, fragmentShaderHandle);

		// Bind attributes
		if(attributes != null) {
			final int size = attributes.length;
			for (int i = 0; i < size; i++) {
				GLES20.glBindAttribLocation(programHandle, i, attributes[i]);
			}
		}

		GLES20.glLinkProgram(programHandle);
		final int[] linkStatus = new int[1];
		GLES20.glGetProgramiv(programHandle, GLES20.GL_LINK_STATUS, linkStatus, 0);

		if(linkStatus[0] == 0) {
			Log.e("AR", "Error compiling program: " + GLES20.glGetProgramInfoLog(programHandle));
			GLES20.glDeleteProgram(programHandle);
			throw new RuntimeException("Could not link shader program.");
		}

		linkedShaders.put(key.toString(), programHandle);
		return programHandle;
	}

	private static int framebufferHandle = -1;
	public static void bindComputeFramebuffer(int targetTextureHandle) {
		if(framebufferHandle < 0) {
			int[] someFBs = new int[1];
			GLES20.glGenFramebuffers(1, someFBs, 0);
			framebufferHandle = someFBs[0];
		}

		GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, framebufferHandle);

    GLES20.glFramebufferTexture2D(GLES20.GL_FRAMEBUFFER, GLES20.GL_COLOR_ATTACHMENT0, GLES20.GL_TEXTURE_2D, targetTextureHandle, 0);

	  if(GLES20.glCheckFramebufferStatus(GLES20.GL_FRAMEBUFFER) != GLES20.GL_FRAMEBUFFER_COMPLETE) {
			throw new RuntimeException("Something went wrong with the framebuffer");
		}
	}

	private static FloatBuffer quadPositions;
	static {
		float[] positions = {
			-1f, 1f,
			-1f, -1f,
			1f, 1f,
			-1f, -1f,
			1f, -1f,
			1f, 1f
		};

		quadPositions = ByteBuffer.allocateDirect(positions.length * Utils.BYTES_PER_FLOAT)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
		quadPositions.put(positions).position(0);
	}

	private static FloatBuffer quadTexCoords;
	static {
		float[] texCoords = {
			0.0f, 1.0f,
			0.0f, 0.0f,
			1.0f, 1.0f,
			0.0f, 0.0f,
			1.0f, 0.0f,
			1.0f, 1.0f,
		};

		quadTexCoords = ByteBuffer.allocateDirect(texCoords.length * Utils.BYTES_PER_FLOAT)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
		quadTexCoords.put(texCoords).position(0);
	}

	public static void runShaderComputationRaw(int programHandle, int positionHandle, int texCoordsHandle) {
		GLES20.glUseProgram(programHandle);
		noGlError();

		quadPositions.position(0);
		GLES20.glVertexAttribPointer(positionHandle, 2, GLES20.GL_FLOAT, false,
				0, quadPositions);
		GLES20.glEnableVertexAttribArray(positionHandle);
		noGlError();

		if(texCoordsHandle > 0) {
			quadTexCoords.position(0);
			GLES20.glVertexAttribPointer(texCoordsHandle, 2, GLES20.GL_FLOAT, false,
					0, quadTexCoords);
			GLES20.glEnableVertexAttribArray(texCoordsHandle);
		}
		noGlError();

		GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, 6);
		noGlError();
	}

	public static void runShaderComputation(int programHandle, int positionHandle, int texCoordsHandle,
			int targetTextureHandle) {
		bindComputeFramebuffer(targetTextureHandle);
		noGlError();

		runShaderComputationRaw(programHandle, positionHandle, texCoordsHandle);

		// and back to default framebuffer
		GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, 0);
		noGlError();
	}

	static class LoadedComputeProgram {
		public int programHandle;
		public int positionHandle;
		public int texCoordsHandle;
	}

	private static HashMap<String, LoadedComputeProgram> compiledComputePrograms = new HashMap<>();
	public static void runShaderComputation(String fragmentShaderCode, int targetTextureHandle) {
		LoadedComputeProgram program = compiledComputePrograms.get(fragmentShaderCode);
		noGlError();

		if(program == null) {
			program = new LoadedComputeProgram();
			program.programHandle = compileShader(computeVertexShader(), fragmentShaderCode,
					new String[] {"a_Position", "a_TexCoordinate"});
			program.positionHandle = GLES20.glGetAttribLocation(program.programHandle, "a_Position");
			program.texCoordsHandle = GLES20.glGetAttribLocation(program.programHandle, "a_TexCoordinate");

			compiledComputePrograms.put(fragmentShaderCode, program);
		}
		noGlError();

		runShaderComputation(program.programHandle, program.positionHandle, program.texCoordsHandle,
				targetTextureHandle);
	}

	public static String computeVertexShader() {
		return
			  "attribute vec2 a_Position;\n"
			+ "attribute vec2 a_TexCoordinate;\n"
      + "varying vec2 v_TexCoordinate;\n"
			+ "void main() {\n"
			+ "   v_TexCoordinate = a_TexCoordinate;\n"
		  + "   gl_Position = vec4(a_Position.x, a_Position.y, 0.0, 1.0);\n"
			+ "}\n";
	}

	public static void close() {
		GLES20.glDeleteFramebuffers(1, new int[]{ framebufferHandle }, 0);

		framebufferHandle = -1;
	}

	public static void noGlError() {
		if(GLES20.glGetError() != 0) {
			throw new RuntimeException("GLError encountered");
		}
	}

	// Returns a static textureId (which receiver must not delete)
	public static class TextInfo {
		public int texture;
		public float width;
		public float height;
	}

	static HashMap<String, TextInfo> renderedTexts = new HashMap<>();

	public static TextInfo renderText(String s) {
		TextInfo info = renderedTexts.get(s);
		if(info != null) return info;

		Paint textPaint = new Paint();
		textPaint.setTextSize(32);
		textPaint.setAntiAlias(true);
		textPaint.setARGB(0xff, 0xff, 0xff, 0xff);

		Rect bounds = new Rect();
		textPaint.getTextBounds(s, 0, s.length(), bounds);

		Bitmap bitmap = Bitmap.createBitmap(bounds.width() + 3, bounds.height() + 10, Bitmap.Config.ARGB_8888);
		Canvas canvas = new Canvas(bitmap);

		bitmap.eraseColor(0);
		canvas.drawText(s, 0, bounds.height(), textPaint);

		int[] textures = new int[1];
		GLES20.glGenTextures(1, textures, 0);
		GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textures[0]);

		GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR_MIPMAP_LINEAR);
		GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
		GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, bitmap, 0);
		GLES20.glGenerateMipmap(GLES20.GL_TEXTURE_2D);
		bitmap.recycle();

		info = new TextInfo();
		info.texture = textures[0];
		info.width = bounds.width();
		info.height = bounds.height();
		renderedTexts.put(s, info);
		return info;
	}
}
