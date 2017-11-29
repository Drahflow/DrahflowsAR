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

public class SpaceMenu implements Geometry {
	private float x, y, z;
	private float[] activationPose;
	private long lastMovementTimestamp;

	private int activeHexagon;

	private static final float SPHERE_SIZE = 0.003f;
	private static final long MILLISECOND = 1000000;

	private static final long ACTIVATION_TIME = 250 * MILLISECOND;
	private static final long OPEN_TIME = 750 * MILLISECOND;
	private static final float TEXT_SCALE = 80;
	private static final float MAINMENU_ALIGN[] = {
		1, 0, -1, -1, 0, 1
	};
	private static final float SUBMENU_ALIGN[] = {
		1, 1, 1, 0, -1,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0
	};

	private GlobalState global;

	private String[] menuTitles = new String[6];
	private ActionCallback[] menuActions = new ActionCallback[30];

	public SpaceMenu(GlobalState global) {
		this.global = global;

		activationPose = null;
		x = 1e6f;
		y = 1e6f;
		z = 1e6f;
	}

	public void setTitle(int i, String title) {
		menuTitles[i] = title;
	}

	public void setAction(int i, int j, ActionCallback callback) {
		menuActions[i * 5 + j] = callback;
	}

	private Geometry off() {
			x = 1e6f;
			y = 1e6f;
			z = 1e6f;
			activationPose = null;
			return null;
	}

	private float distance(float dx, float dy, float dz) {
		return (float)Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

	@Override
	public Geometry onPointer(PointerEvent e) {
		if(!e.active) return off();

		float dist = distance(e.x - x, e.y - y, e.z - z);

		if(e.focusedOn == null) {
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
				activeHexagon = 0;
				return this;
			}

			return null;
		} else if(e.focusedOn == this) {
			if(dist < SPHERE_SIZE) {
				activeHexagon = 0;
			} else if(activeHexagon == 0) {
				int hexagonIndex = 1;
				for(float a = 0; a < 2 * Math.PI - 0.0001; a += Math.PI / 3) {
					final float b = (float)(a + Math.PI / 3);

					final float cx = (float)((2 + 2 / Math.sqrt(3)) * (Math.cos(a) + Math.cos(b)) / 2);
					final float cy = (float)((2 + 2 / Math.sqrt(3)) * -(Math.sin(a) + Math.sin(b)) / 2);

					final float subDist = distance(x + SPHERE_SIZE * cx - e.x, y + SPHERE_SIZE * cy - e.y, z - e.z);
					if(subDist < SPHERE_SIZE) {
						activeHexagon = hexagonIndex;
					}
					++hexagonIndex;
				}
			} else if(activeHexagon > 0) {
				final float a = (float)((activeHexagon - 1) * Math.PI / 3);
				final float b = (float)(a + Math.PI / 3);
				final float cx = (float)((2 + 2 / Math.sqrt(3)) * (Math.cos(a) + Math.cos(b)) / 2);
				final float cy = (float)((2 + 2 / Math.sqrt(3)) * -(Math.sin(a) + Math.sin(b)) / 2);

				int actionIndex = 0;
				for(float c = (float)(a + 4 * Math.PI / 3); c < a + 9 * Math.PI / 3 - 0.0001; c += Math.PI / 3) {
					final float d = (float)(c + Math.PI / 3);

					final float dx = cx + (float)((2 + 2 / Math.sqrt(3)) * (Math.cos(c) + Math.cos(d)) / 2);
					final float dy = cy + (float)((2 + 2 / Math.sqrt(3)) * -(Math.sin(c) + Math.sin(d)) / 2);

					final float subDist = distance(x + SPHERE_SIZE * dx - e.x, y + SPHERE_SIZE * dy - e.y, z - e.z);
					if(subDist < SPHERE_SIZE) {
						Log.e("AR", "SpaceMenu Action Selected (" + activeHexagon + "," + actionIndex + ")");
						ActionCallback action = menuActions[(activeHexagon - 1) * 5 + actionIndex];
						if(action != null) {
							action.selected(activationPose, x, y, z);
							return off();
						}
					}
					++actionIndex;
				}
			}

			if(activeHexagon == 0 && dist > SPHERE_SIZE * 4) {
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

			lines.add((float)((Math.cos(a) + Math.cos(b)) / 2));
			lines.add((float)(-(Math.sin(a) + Math.sin(b)) / 2));
			lines.add((float)(0));
			lines.add((float)((1 + 2 / Math.sqrt(3)) * (Math.cos(a) + Math.cos(b)) / 2));
			lines.add((float)((1 + 2 / Math.sqrt(3)) * -(Math.sin(a) + Math.sin(b)) / 2));
			lines.add((float)(0));
		}

		// outer hexagons + extruding spokes
		for(float a = 0; a < 2 * Math.PI - 0.0001; a += Math.PI / 3) {
			final float b = (float)(a + Math.PI / 3);

			final float cx = (float)((2 + 2 / Math.sqrt(3)) * (Math.cos(a) + Math.cos(b)) / 2);
			final float cy = (float)((2 + 2 / Math.sqrt(3)) * -(Math.sin(a) + Math.sin(b)) / 2);

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

				lines.add((float)(cx + (Math.cos(c) + Math.cos(d)) / 2));
				lines.add((float)(cy + -(Math.sin(c) + Math.sin(d)) / 2));
				lines.add((float)(0));
				lines.add((float)(cx + (1 + 2 / Math.sqrt(3)) * (Math.cos(c) + Math.cos(d)) / 2));
				lines.add((float)(cy + (1 + 2 / Math.sqrt(3)) * -(Math.sin(c) + Math.sin(d)) / 2));
				lines.add((float)(0));
			}
		}

		linePositions = ByteBuffer.allocateDirect(lines.size() * Utils.BYTES_PER_FLOAT)
			.order(ByteOrder.nativeOrder()).asFloatBuffer();
		final float[] tmp = new float[lines.size()];
		for(int i = 0; i < lines.size(); ++i) tmp[i] = lines.get(i);
		linePositions.put(tmp).position(0);
	}

	private static FloatBuffer quadPositions;
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
		Matrix.setIdentityM(modelMatrix, 0);
		Matrix.translateM(modelMatrix, 0, x, y, z);
		float scale = SPHERE_SIZE;
		Matrix.scaleM(modelMatrix, 0, scale, scale, scale);

		// TODO: Rotate (and maybe scale) according to activation matrix

		Matrix.multiplyMM(mvpsMatrix, 0, vpsMatrix, 0, modelMatrix, 0);
		Utils.noGlError();
		GLES20.glUniformMatrix4fv(mvpsMatrixHandle, 1, false, mvpsMatrix, 0);

		// Actually draw
		GLES20.glDrawArrays(GLES20.GL_LINES, 0, 24);

		if(activeHexagon > 0) {
			GLES20.glDrawArrays(GLES20.GL_LINES, 24 + (activeHexagon - 1) * 22, 22);

			for(int i = 0; i < 5; ++i) {
				ActionCallback action = menuActions[(activeHexagon - 1) * 5 + i];

				if(action == null) continue;

				final float a = (float)((activeHexagon - 1) * Math.PI / 3);
				final float b = (float)(a + Math.PI / 3);

				final float cx = (float)((2 + 2 / Math.sqrt(3)) * (Math.cos(a) + Math.cos(b)) / 2);
				final float cy = (float)((2 + 2 / Math.sqrt(3)) * -(Math.sin(a) + Math.sin(b)) / 2);

				final float c = (float)((activeHexagon + 3 + i) * Math.PI / 3);
				final float d = (float)(c + Math.PI / 3);

				final float dx = cx + (float)((1.2 + 2 / Math.sqrt(3)) * (Math.cos(c) + Math.cos(d)) / 2);
				final float dy = cy + (float)((1.2 + 2 / Math.sqrt(3)) * -(Math.sin(c) + Math.sin(d)) / 2);

				renderText(vpsMatrix, action.getTitle(), dx, dy, SUBMENU_ALIGN[(activeHexagon - 1) * 5 + i]);
			}
		} else {
			for(int i = 0; i < menuTitles.length; ++i) {
				if(menuTitles[i] == null) continue;

				final float a = (float)(i * Math.PI / 3);
				final float b = (float)(a + Math.PI / 3);

				final float cx = (float)((1.2 + 2 / Math.sqrt(3)) * (Math.cos(a) + Math.cos(b)) / 2);
				final float cy = (float)((1.2 + 2 / Math.sqrt(3)) * -(Math.sin(a) + Math.sin(b)) / 2);

				renderText(vpsMatrix, menuTitles[i], cx, cy, MAINMENU_ALIGN[i]);
			}
		}
	}

	private void renderText(float[] vpsMatrix, String string, float tx, float ty, float alignX) {
		final int linkedShaderHandle = Utils.compileShader(getTextVertexShader(), getTextFragmentShader(),
				new String[] {"a_Position"});
		Utils.noGlError();
		final int positionHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_Position");
		final int mvpsMatrixHandle = GLES20.glGetUniformLocation(linkedShaderHandle, "u_MVPSMatrix");
		GLES20.glUseProgram(linkedShaderHandle);

		Utils.TextInfo text = Utils.renderText(string);

		Matrix.setIdentityM(modelMatrix, 0);
		Matrix.translateM(modelMatrix, 0, x, y, z);
		float scale = SPHERE_SIZE;
		Matrix.scaleM(modelMatrix, 0, scale, scale, scale);
		Matrix.translateM(modelMatrix, 0, tx + alignX * text.width / TEXT_SCALE, ty, 0);
		Matrix.scaleM(modelMatrix, 0, text.width / TEXT_SCALE, text.height / TEXT_SCALE, 1);

		Matrix.multiplyMM(mvpsMatrix, 0, vpsMatrix, 0, modelMatrix, 0);
		GLES20.glUniformMatrix4fv(mvpsMatrixHandle, 1, false, mvpsMatrix, 0);
		Utils.noGlError();

		quadPositions.position(0);
		Utils.noGlError();
		GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
				0, quadPositions);
		Utils.noGlError();
		GLES20.glEnableVertexAttribArray(positionHandle);
		Utils.noGlError();

		GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, text.texture);
		Utils.noGlError();
		int textureHandle = GLES20.glGetUniformLocation(linkedShaderHandle , "u_Texture");
		GLES20.glUniform1i(textureHandle, 0);
		Utils.noGlError();

		GLES20.glEnable(GLES20.GL_BLEND);
		Utils.noGlError();
		GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA);
		Utils.noGlError();

		GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, 6);
		Utils.noGlError();

		GLES20.glDisable(GLES20.GL_BLEND);
		Utils.noGlError();
	}

	protected String getTextVertexShader() {
		// Define our per-pixel lighting shader.
		final String shader =
				"uniform mat4 u_MVPSMatrix;      \n"		// A constant representing the combined model/view/projection matrix.
			+ "attribute vec4 a_Position;     \n"		// Per-vertex position information we will pass in.
			+ "varying vec2 v_TexCoordinate;  \n"		// This will be passed into the fragment shader.

			// The entry point for our vertex shader.
			+ "void main()                                                \n"
			+ "{                                                          \n"
			+ "   gl_Position = u_MVPSMatrix * a_Position;                 \n"
			+ "   v_TexCoordinate = vec2(1.0 + a_Position.x, 1.0 - a_Position.y) / 2.0;\n"
			+ "}                                                          \n";

		return shader;
	}

	public String getTextFragmentShader() {
		final String shader =
				"precision mediump float;\n" // Set the default precision to medium.
			+ "uniform sampler2D u_Texture;\n"
			+ "varying vec2 v_TexCoordinate;  \n"		// This will be passed into the fragment shader.
			+ "void main() {\n"
			+ "   gl_FragColor = vec4(1, 1, 1, texture2D(u_Texture, v_TexCoordinate).r); \n"
			+ "}\n";

		return shader;
	}

	public interface ActionCallback {
		public String getTitle();
		public void selected(float[] activationPose, float x, float y, float z);
	}
}
