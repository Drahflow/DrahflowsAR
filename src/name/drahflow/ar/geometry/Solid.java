package name.drahflow.ar.geometry;

public class Solid {
	private Texture texture = Constants.RED;

	public void setTexture(Texture t) {
		texture = t;
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

	protected String getFragmentShader() {
		return texture.getFragmentShader();
	}
}
