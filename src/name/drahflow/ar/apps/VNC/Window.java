package name.drahflow.ar.apps.VNC;

import name.drahflow.ar.geometry.Geometry;
import name.drahflow.ar.geometry.PointerEvent;
import name.drahflow.ar.Utils;

import android.opengl.GLES20;
import android.opengl.GLUtils;
import android.opengl.Matrix;
import android.util.Log;
import android.view.View;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.content.Context;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

public class Window implements Geometry {
	VncCanvas vncCanvas;

	static final String FIT_SCREEN_NAME = "FIT_SCREEN";
	static final String TOUCH_ZOOM_MODE = "TOUCH_ZOOM_MODE";
	static final String TOUCHPAD_MODE = "TOUCHPAD_MODE";

	private Bitmap textureBitmap;
	private Canvas textureCanvas;

	public Window(Context ctx, String host, int port) {
		ConnectionBean con = new ConnectionBean();
		con.setAddress(host);
		con.setPort(port);
		con.setForceFull(BitmapImplHint.FULL);

		vncCanvas = new VncCanvas(ctx, con, new Runnable() {
			@Override public void run() {
				textureBitmap = Bitmap.createBitmap(
					vncCanvas.getImageWidth(),
					vncCanvas.getImageHeight(),
					// 1024, 1024,
					Bitmap.Config.ARGB_8888
				);
				textureCanvas = new Canvas(textureBitmap);
				vncCanvas.layout(0, 0, vncCanvas.getImageWidth(), vncCanvas.getImageHeight());
			}
		});
	}

	// @Override
	// protected void onStop() {
	// 	vncCanvas.disableRepaints();
	// 	super.onStop();
	// }

	// @Override
	// protected void onRestart() {
	// 	vncCanvas.enableRepaints();
	// 	super.onRestart();
	// }
	
	@Override
	public Geometry onPointer(PointerEvent e) {
		// TODO:
		// * Window movements + rotations
		// * Sending mouse events to VNC content
		return null;
	}
	
	private static float[] modelMatrix = new float[16];
	private static float[] mvpsMatrix = new float[16];

	public void render(float[] vpsMatrix) {
		if(textureCanvas == null) return;

		vncCanvas.draw(textureCanvas);

		Matrix.setIdentityM(modelMatrix, 0);

		// TODO: Abstract this away into a global shading loading / uniform setter

		final int linkedShaderHandle = Utils.compileShader(getVertexShader(), getFragmentShader(),
				new String[] {"a_Position", "a_TexCoordinate"});

		// Set our per-vertex program.
		GLES20.glUseProgram(linkedShaderHandle);
		final int mvpsMatrixHandle = GLES20.glGetUniformLocation(linkedShaderHandle, "u_MVPSMatrix");
		final int positionHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_Position");
		final int texCoordsHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "a_TexCoordinate");
		final int texSamplerHandle = GLES20.glGetAttribLocation(linkedShaderHandle, "u_TexImage");

		// Pass in the position information
		quadPositions.position(0);
		GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
				0, quadPositions);
		GLES20.glEnableVertexAttribArray(positionHandle);

		// Pass in texture information
		quadTexCoords.position(0);
		GLES20.glVertexAttribPointer(texCoordsHandle, 2, GLES20.GL_FLOAT, false,
				0, quadTexCoords);
		GLES20.glEnableVertexAttribArray(texCoordsHandle);

		GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
		GLES20.glUniform1i(texSamplerHandle, 0);

		// TODO: Initialize once
		final int[] vncTextures = new int[1];
    // generate one texture pointer...
    GLES20.glGenTextures(1, vncTextures, 0);
    //...and bind it to our array
    GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, vncTextures[0]);

    GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR_MIPMAP_LINEAR);
    GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);

    // different possible texture parameters, e.g. GLES20.GL_CLAMP_TO_EDGE
    GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_REPEAT);
    GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_REPEAT);

    // use the Android GLUtils to specify a two-dimensional texture image from our bitmap
    GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, textureBitmap, 0);
		GLES20.glGenerateMipmap(GLES20.GL_TEXTURE_2D);

		// FIXME: Should be unscaled (and VNC window stuffed under generic transform)
		// Matrix.scaleM(modelMatrix, 0, 0.064f, 0.048f, 0.01f);

		Matrix.multiplyMM(mvpsMatrix, 0, vpsMatrix, 0, modelMatrix, 0);
		Utils.noGlError();
		GLES20.glUniformMatrix4fv(mvpsMatrixHandle, 1, false, mvpsMatrix, 0);

		// Actually draw
		GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, quadPositions.limit() / 3);

		GLES20.glDeleteTextures(1, vncTextures, 0);
	}

	private static final FloatBuffer quadPositions;
	private static final FloatBuffer quadTexCoords;
	static {
		float[] positions = {
			0, 0, 0,   1, 0, 0,   1,  1, 0,
			0, 1, 0,   0, 0, 0,   1,  1, 0,
		};

		quadPositions = ByteBuffer.allocateDirect(positions.length * Utils.BYTES_PER_FLOAT)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
		quadPositions.put(positions).position(0);

		float[] texCoords = {
			0, 1,    1, 1,     1, 0,
			0, 0,    0, 1,     1, 0
		};

		quadTexCoords = ByteBuffer.allocateDirect(texCoords.length * Utils.BYTES_PER_FLOAT)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
		quadTexCoords.put(texCoords).position(0);
	}

	protected String getVertexShader() {
		final String perPixelVertexShader =
				"uniform mat4 u_MVPSMatrix;      \n"
			+ "attribute vec4 a_Position;     \n"
			+ "attribute vec2 a_TexCoordinate;\n"
			+ "varying vec2 v_TexCoordinate;  \n"
			+ "void main()                                \n"
			+ "{                                          \n"
			+ "   v_TexCoordinate = a_TexCoordinate;      \n"
			+ "   gl_Position = u_MVPSMatrix * a_Position; \n"
			+ "}                                          \n";

		return perPixelVertexShader;
	}

	protected String getFragmentShader() {
		final String perPixelFragmentShader =
				"precision mediump float;\n"
			+ "uniform sampler2D u_TexImage;\n"
			+ "varying vec2 v_TexCoordinate;\n"
			+ "void main() {\n"
			+ "   gl_FragColor = texture2D(u_TexImage, v_TexCoordinate); \n"
			+ "}\n";

		return perPixelFragmentShader;
	}
}
