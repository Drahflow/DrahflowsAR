package name.drahflow.ar.geometry;

import android.opengl.GLES20;
import android.opengl.Matrix;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import name.drahflow.ar.Utils;
import name.drahflow.ar.GlobalState;

public class OverlayText implements Geometry {
	public static final long DISPLAY_DURATION = 5000000000l; // nanoseconds
	private GlobalState global;
	private float x, y, size;
	private String string;
	private long shownAt;

	public OverlayText(GlobalState global, String s, float x, float y) {
		this.global = global;
		this.string = s;
		this.x = x;
		this.y = y;
	}

	public void show() {
		shownAt = System.nanoTime();
	}

	public boolean isShowing() {
		return shownAt + DISPLAY_DURATION > System.nanoTime();
	}

	private static FloatBuffer quadPositions;
	private static FloatBuffer quadTexCoords;

	static {
		final float[] positions = {
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

		final float[] texCoords = {
			// Front face
			0.0f, 1.0f,
			1.0f, 1.0f,
			1.0f, 0.0f,
			0.0f, 0.0f,
			0.0f, 1.0f,
			1.0f, 0.0f,
		};

		quadTexCoords = ByteBuffer.allocateDirect(texCoords.length * Utils.BYTES_PER_FLOAT)
			.order(ByteOrder.nativeOrder()).asFloatBuffer();
		quadTexCoords.put(texCoords).position(0);
	}

	private static float[] inversePoseMatrix = new float[16];
	private static float[] poseMatrix = new float[16];
	private static float[] fixedModelMatrix = new float[16];
	private static float[] modelMatrix = new float[16];
	private static float[] mvpsMatrix = new float[16];

	public void render(float[] vpsMatrix) {
		if(!isShowing()) return;

		// TODO: Abstract this away into a global shading loading / uniform setter

		final int linkedShaderHandle = Utils.compileShader(getVertexShader(), getFragmentShader(),
				new String[] {"a_Position", "a_TexCoordinate"});

		// Set our per-vertex program.
		GLES20.glUseProgram(linkedShaderHandle);
		final int mvpsMatrixHandle = GLES20.glGetUniformLocation(linkedShaderHandle, "u_MVPSMatrix");
		final int positionHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_Position");
		final int texCoordsHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_TexCoordinate");

		// Pass in the position information
		quadPositions.position(0);
		GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
				0, quadPositions);
		GLES20.glEnableVertexAttribArray(positionHandle);

		// Pass in the texture coordinate information
		if(texCoordsHandle >= 0) {
			quadTexCoords.position(0);
			GLES20.glVertexAttribPointer(texCoordsHandle, 2, GLES20.GL_FLOAT, false,
					0, quadTexCoords);
			GLES20.glEnableVertexAttribArray(texCoordsHandle);
		}

		Utils.TextInfo text = Utils.renderText(string);

		// Model transformations
		Matrix.setIdentityM(modelMatrix, 0);
		Matrix.translateM(modelMatrix, 0, x, y, -0.2f);
		Matrix.scaleM(modelMatrix, 0, 0.00005f * text.width, 0.00005f * text.height, 0.00005f);

		global.view.getCachedPose(poseMatrix);
		Matrix.invertM(inversePoseMatrix, 0, poseMatrix, 0);
		Matrix.multiplyMM(fixedModelMatrix, 0, inversePoseMatrix, 0, modelMatrix, 0);

		Matrix.multiplyMM(mvpsMatrix, 0, vpsMatrix, 0, fixedModelMatrix, 0);
		Utils.noGlError();
		GLES20.glUniformMatrix4fv(mvpsMatrixHandle, 1, false, mvpsMatrix, 0);

		GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, text.texture);
		int textureHandle = GLES20.glGetUniformLocation(linkedShaderHandle , "u_Texture");
		GLES20.glUniform1i(textureHandle, 0);

		GLES20.glEnable(GLES20.GL_BLEND);
		GLES20.glDisable(GLES20.GL_DEPTH_TEST);
		GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA);

		GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, 6);

		GLES20.glEnable(GLES20.GL_DEPTH_TEST);
		GLES20.glDisable(GLES20.GL_BLEND);
	}

	@Override public Geometry onPointer(PointerEvent e) { return null; }

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
			+ "   gl_Position = u_MVPSMatrix * a_Position;                 \n"
			+ "}                                                          \n";

		return perPixelVertexShader;
	}

	public String getFragmentShader() {
		final String perPixelFragmentShader =
				"precision mediump float;\n" // Set the default precision to medium.
			+ "uniform sampler2D u_Texture;\n"
			+ "varying vec2 v_TexCoordinate;  \n"		// This will be passed into the fragment shader.
			+ "void main() {\n"
			+ "   gl_FragColor = vec4(1, 1, 1, texture2D(u_Texture, v_TexCoordinate).r); \n"
			+ "}\n";

		return perPixelFragmentShader;
	}
}
