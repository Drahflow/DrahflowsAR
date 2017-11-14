package name.drahflow.ar.geometry;

import android.opengl.GLES10;
import android.opengl.GLES20;
import android.opengl.Matrix;
import android.util.Log;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;

import name.drahflow.ar.GlobalState;
import name.drahflow.ar.Utils;

public class SpaceSelector implements Geometry {
	private float sx, sy, sz;
	private float ex, ey, ez;
	private float[] activationPose;
	private long lastMovementTimestamp;
	private boolean activated;

	private static final float SPHERE_SIZE = 0.003f;
	private static final long MILLISECOND = 1000000;

	private static final long ACTIVATION_TIME = 250 * MILLISECOND;
	private static final long OPEN_TIME = 1500 * MILLISECOND;

	private GlobalState global;

	private ActionCallback selectionAction;

	public SpaceSelector(GlobalState global, float[] activationPose, float sx, float sy, float sz,
			ActionCallback callback) {
		this.global = global;
		this.activationPose = activationPose;
		this.selectionAction = callback;
		this.sx = sx;
		this.sy = sy;
		this.sz = sz;
	}

	private Geometry off() {
		// FIXME: Remove from scene
		activationPose = null;
		activated = false;
		return null;
	}

	private float distance(float dx, float dy, float dz) {
		return (float)Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

	@Override
	public Geometry onPointer(PointerEvent e) {
		if(!e.active) return off();
		if(activationPose == null) return off();

		float dist = distance(e.x - ex, e.y - ey, e.z - ez);

		if(dist > SPHERE_SIZE) {
			lastMovementTimestamp = e.timestamp;
			ex = e.x; ey = e.y; ez = e.z;
			activated = false;
		}

		if(!activated && lastMovementTimestamp + ACTIVATION_TIME < e.timestamp) {
			lastMovementTimestamp = e.timestamp - ACTIVATION_TIME;
			ex = e.x; ey = e.y; ez = e.z;
			activated = true;
		}

		if(activated && lastMovementTimestamp + OPEN_TIME < e.timestamp) {
			selectionAction.selected(activationPose, sx, sy, sz, ex, ey, ez);
			return off();
		}

		return this;
	}

	@Override
	public void render(float[] vpsMatrix) {
		if(!activated) return;

		long now = System.nanoTime();

		if(lastMovementTimestamp + ACTIVATION_TIME < now) {
			renderActivatedMenu(vpsMatrix, now);
		}

		renderSelectedSpace(vpsMatrix);
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
		Matrix.translateM(modelMatrix, 0, ex, ey, ez);
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

	private void renderSelectedSpace(float[] vpsMatrix) {
		// TODO: Render some selection into space
	}

	public interface ActionCallback {
		public void selected(float[] activationPose,
				float sx, float sy, float sz, float ex, float ey, float ez);
	}
}
