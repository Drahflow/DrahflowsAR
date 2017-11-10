package name.drahflow.ar.geometry;

import android.opengl.GLES20;
import android.opengl.Matrix;
import android.util.Log;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;

import name.drahflow.ar.GlobalState;
import name.drahflow.ar.Utils;

public class SpaceMenu implements Geometry {
	private float x, y, z;
	private float[] activationPose;
	private long lastMovementTimestamp;

	private static final float SPHERE_SIZE = 0.003f;
	private static final long MILLISECOND = 1000000;

	private static final long ACTIVATION_TIME = 250 * MILLISECOND;
	private static final long OPEN_TIME = 750 * MILLISECOND;

	private GlobalState global;

	public SpaceMenu(GlobalState global) {
		this.global = global;

		activationPose = null;
		x = 1e6f;
		y = 1e6f;
		z = 1e6f;
	}

	private Geometry off() {
			x = 1e6f;
			y = 1e6f;
			z = 1e6f;
			activationPose = null;
			return null;
	}

	@Override
	public Geometry onPointer(PointerEvent e) {
		if(!e.active) return off();

		if(e.focusedOn == null) {
			float dx = e.x - x;
			float dy = e.y - y;
			float dz = e.z - z;

			float dist = (float)Math.sqrt(dx * dx + dy * dy + dz * dz);

			if(dist > SPHERE_SIZE) {
				lastMovementTimestamp = e.timestamp;
				x = e.x; y = e.y; z = e.z;
				return null;
			}

			if(lastMovementTimestamp + ACTIVATION_TIME < e.timestamp) {
				activationPose = new float[16];
				global.view.getCachedPose(activationPose);

				lastMovementTimestamp = e.timestamp - ACTIVATION_TIME;
				x = e.x; y = e.y; z = e.z;
				return this;
			}

			return null;
		} else if(e.focusedOn == this) {
			// FIXME handle things here
			if(lastMovementTimestamp + 2000000000l < e.timestamp) {
				return off();
			}

			return this;
		}

		return null;
	}

	@Override
	public void render(float[] vpsMatrix) {
		if(activationPose == null) return;

		long now = System.nanoTime();
		if(lastMovementTimestamp + OPEN_TIME < now) {
			renderOpenMenu(vpsMatrix);
		} else if(lastMovementTimestamp + ACTIVATION_TIME < now) {
			renderActivatedMenu(vpsMatrix, now);
		}
	}

	private float[] modelMatrix = new float[16];
	private float[] mvpsMatrix = new float[16];

	private static FloatBuffer pointPositions;
	static {
		final float[] positions = {
       1.5f,  0.5f * (float)Math.sqrt(3), 0f,
       0f  ,  1f   * (float)Math.sqrt(3), 0f,
      -1.5f,  0.5f * (float)Math.sqrt(3), 0f,
      -1.5f, -0.5f * (float)Math.sqrt(3), 0f,
       0f  , -1f   * (float)Math.sqrt(3), 0f,
       1.5f, -0.5f * (float)Math.sqrt(3), 0f,
		};

		pointPositions = ByteBuffer.allocateDirect(positions.length * Utils.BYTES_PER_FLOAT)
			.order(ByteOrder.nativeOrder()).asFloatBuffer();
		pointPositions.put(positions).position(0);
	}

	private static FloatBuffer linePositions;
	static {
		ArrayList<Float> lines = new ArrayList<>();

		// central hexagon
		for(float a = 0; a < 2 * Math.PI - 0.0001; a += Math.PI / 3) {
			final float b = (float)(a + Math.PI / 3);

			lines.add((float)(Math.cos(a)));
			lines.add((float)(-Math.sin(a)));
			lines.add((float)(0));
			lines.add((float)(Math.cos(b)));
			lines.add((float)(-Math.sin(b)));
			lines.add((float)(0));
		}

		// central spokes
		for(float a = 0; a < 2 * Math.PI - 0.0001; a += Math.PI / 3) {
			final float b = (float)(a + Math.PI / 3);

			lines.add((float)(Math.sqrt(3) / 2 * (Math.cos(a) + Math.cos(b)) / 2));
			lines.add((float)(Math.sqrt(3) / 2 * -(Math.sin(a) + Math.sin(b)) / 2));
			lines.add((float)(0));
			lines.add((float)((1 + Math.sqrt(3) / 2) * (Math.cos(a) + Math.cos(b)) / 2));
			lines.add((float)((1 + Math.sqrt(3) / 2) * -(Math.sin(a) + Math.sin(b)) / 2));
			lines.add((float)(0));
		}

		// outer hexagons + extruding spokes
		for(float a = 0; a < 2 * Math.PI - 0.0001; a += Math.PI / 3) {
			final float b = (float)(a + Math.PI / 3);

			float cx = (float)((1 + 2 * Math.sqrt(3) / 2) * (Math.cos(a) + Math.cos(b)) / 2);
			float cy = (float)((1 + 2 * Math.sqrt(3) / 2) * -(Math.sin(a) + Math.sin(b)) / 2);

			// hexagon line
			for(float c = 0; c < 2 * Math.PI - 0.0001; c += Math.PI / 3) {
				final float d = (float)(c + Math.PI / 3);

				lines.add((float)(cx + Math.cos(c)));
				lines.add((float)(cy + -Math.sin(c)));
				lines.add((float)(0));
				lines.add((float)(cx + Math.cos(d)));
				lines.add((float)(cy + -Math.sin(d)));
				lines.add((float)(0));
			}

			// spokes
			for(float c = (float)(a + 4 * Math.PI / 3); c < a + 9 * Math.PI / 3 - 0.0001; c += Math.PI / 3) {
				final float d = (float)(c + Math.PI / 3);

				lines.add((float)(cx + Math.sqrt(3) / 2 * (Math.cos(c) + Math.cos(d)) / 2));
				lines.add((float)(cy + Math.sqrt(3) / 2 * -(Math.sin(c) + Math.sin(d)) / 2));
				lines.add((float)(0));
				lines.add((float)(cx + (1 + Math.sqrt(3) / 2) * (Math.cos(c) + Math.cos(d)) / 2));
				lines.add((float)(cy + (1 + Math.sqrt(3) / 2) * -(Math.sin(c) + Math.sin(d)) / 2));
				lines.add((float)(0));
			}
		}

		linePositions = ByteBuffer.allocateDirect(lines.size() * Utils.BYTES_PER_FLOAT)
			.order(ByteOrder.nativeOrder()).asFloatBuffer();
		final float[] tmp = new float[lines.size()];
		for(int i = 0; i < lines.size(); ++i) tmp[i] = lines.get(i);
		linePositions.put(tmp).position(0);
	}

	private void renderActivatedMenu(float[] vpsMatrix, long now) {
		Matrix.setIdentityM(modelMatrix, 0);

		// TODO: Abstract this away into a global shading loading / uniform setter

		final int linkedShaderHandle = Utils.compileShader(getVertexShader(), getFragmentShader(),
				new String[] {"a_Position"});

		// Set our per-vertex program.
		GLES20.glUseProgram(linkedShaderHandle);
		final int mvpsMatrixHandle = GLES20.glGetUniformLocation(linkedShaderHandle, "u_MVPSMatrix");
		final int positionHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_Position");

		// Pass in the position information
		pointPositions.position(0);
		GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
				0, pointPositions);
		GLES20.glEnableVertexAttribArray(positionHandle);

		// Model transformations
		Matrix.translateM(modelMatrix, 0, x, y, z);
		float scale = SPHERE_SIZE * (lastMovementTimestamp - now + OPEN_TIME) / (OPEN_TIME - ACTIVATION_TIME);
		Matrix.scaleM(modelMatrix, 0, scale, scale, scale);

		// TODO: Rotate (and maybe scale) according to activation matrix

		Matrix.multiplyMM(mvpsMatrix, 0, vpsMatrix, 0, modelMatrix, 0);
		Utils.noGlError();
		GLES20.glUniformMatrix4fv(mvpsMatrixHandle, 1, false, mvpsMatrix, 0);

		// Actually draw
		GLES20.glDrawArrays(GLES20.GL_POINTS, 0, 6);
	}

	protected String getVertexShader() {
		// Define our per-pixel lighting shader.
		final String perPixelVertexShader =
				"uniform mat4 u_MVPSMatrix;      \n"		// A constant representing the combined model/view/projection matrix.
			+ "attribute vec4 a_Position;     \n"		// Per-vertex position information we will pass in.

			// The entry point for our vertex shader.
			+ "void main()                                 \n"
			+ "{                                           \n"
			+ "   gl_Position = u_MVPSMatrix * a_Position; \n"
			+ "}                                           \n";

		return perPixelVertexShader;
	}

	public String getFragmentShader() {
		final String perPixelFragmentShader =
				"precision mediump float;\n" // Set the default precision to medium.
			+ "void main() {\n"
			+ "  gl_FragColor = vec4(1, 1, 1, 0.0);\n"
			+ "}\n";

		return perPixelFragmentShader;
	}

	private void renderOpenMenu(float[] vpsMatrix) {
		Matrix.setIdentityM(modelMatrix, 0);

		// TODO: Abstract this away into a global shading loading / uniform setter

		final int linkedShaderHandle = Utils.compileShader(getVertexShader(), getFragmentShader(),
				new String[] {"a_Position"});

		// Set our per-vertex program.
		GLES20.glUseProgram(linkedShaderHandle);
		final int mvpsMatrixHandle = GLES20.glGetUniformLocation(linkedShaderHandle, "u_MVPSMatrix");
		final int positionHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_Position");

		// Pass in the position information
		linePositions.position(0);
		GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
				0, linePositions);
		GLES20.glEnableVertexAttribArray(positionHandle);

		// Model transformations
		Matrix.translateM(modelMatrix, 0, x, y, z);
		float scale = SPHERE_SIZE;
		Matrix.scaleM(modelMatrix, 0, scale, scale, scale);

		// TODO: Rotate (and maybe scale) according to activation matrix

		Matrix.multiplyMM(mvpsMatrix, 0, vpsMatrix, 0, modelMatrix, 0);
		Utils.noGlError();
		GLES20.glUniformMatrix4fv(mvpsMatrixHandle, 1, false, mvpsMatrix, 0);

		// Actually draw
		GLES20.glDrawArrays(GLES20.GL_LINES, 0, 156);
	}
}
