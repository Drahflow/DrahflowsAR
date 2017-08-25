package name.drahflow.ar.geometry;

import android.opengl.GLES20;
import android.opengl.Matrix;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import name.drahflow.ar.Utils;

public class Cube extends Solid implements Geometry {
	private float x, y, z, size;

	public Cube(float x, float y, float z, float size) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.size = size;
	}

	private static FloatBuffer cubePositions;
	private static FloatBuffer cubeTexCoords;

	static {
		final float[] positions = {
			-1, -1, -1,   1, -1, -1,
			1,  -1, -1,   1,  1, -1,
			1,   1, -1,  -1,  1, -1,
			-1,  1, -1,  -1, -1, -1,

			-1, -1, -1,  -1, -1,  1,
			1,  -1, -1,   1, -1,  1,
			1,   1, -1,   1,  1,  1,
			-1,  1, -1,  -1,  1,  1,

			-1, -1,  1,   1, -1,  1,
			1,  -1,  1,   1,  1,  1,
			1,   1,  1,  -1,  1,  1,
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

	private static float[] modelMatrix = new float[16];
	private static float[] mvpsMatrix = new float[16];

	public void render(float[] vpsMatrix) {
		Matrix.setIdentityM(modelMatrix, 0);

		// TODO: Abstract this away into a global shading loading / uniform setter

		final int linkedShaderHandle = Utils.compileShader(getVertexShader(), getFragmentShader(),
				new String[] {"a_Position", "a_TexCoordinate"});

		// Set our per-vertex program.
		GLES20.glUseProgram(linkedShaderHandle);
		final int mvpsMatrixHandle = GLES20.glGetUniformLocation(linkedShaderHandle, "u_MVPSMatrix");
		final int positionHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_Position");
		final int texCoordsHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_TexCoordinate");

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

		// Model transformations
		Matrix.translateM(modelMatrix, 0, x, y, z);
		Matrix.scaleM(modelMatrix, 0, size, size, size);

		Matrix.multiplyMM(mvpsMatrix, 0, vpsMatrix, 0, modelMatrix, 0);
		Utils.noGlError();
		GLES20.glUniformMatrix4fv(mvpsMatrixHandle, 1, false, mvpsMatrix, 0);

		// Actually draw
		GLES20.glDrawArrays(GLES20.GL_LINES, 0, 24);
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

	private String getFragmentShader() {
		final String perPixelFragmentShader =
				"precision mediump float;\n" // Set the default precision to medium.
			+ "varying vec2 v_TexCoordinate;\n"
			+ "void main() {\n"
			// + "   gl_FragColor = vec4(v_TexCoordinate.x, v_TexCoordinate.y, 1.0, 0.0); \n"
			+ "   gl_FragColor = vec4(0.0 + v_TexCoordinate.x, 1, 0, 0.0); \n"
			+ "}\n";

		return perPixelFragmentShader;
	}
}
