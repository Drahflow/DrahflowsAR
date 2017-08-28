package name.drahflow.ar.geometry;

public class Color implements Texture {
	private float r, g, b;

	public Color(float r, float g, float b) {
		this.r = r;
		this.g = g;
		this.b = b;
	}

	public String getFragmentShader() {
		final String perPixelFragmentShader =
				"precision mediump float;\n" // Set the default precision to medium.
			+ "void main() {\n"
			+ String.format("gl_FragColor = vec4(%f, %f, %f, 0.0);\n", r, g, b)
			+ "}\n";

		return perPixelFragmentShader;
	}
}
