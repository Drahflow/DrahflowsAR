package name.drahflow.ar.geometry;

import android.opengl.Matrix;

public class Translation implements Geometry {
	private float x, y, z;

	private Geometry subgraph;
	private float[] subgraphMatrix = new float[16];

	public void setSubgraph(Geometry g) {
		subgraph = g;
	}

	public void setTranslation(float[] xyz) {
		x = xyz[0];
		y = xyz[1];
		z = xyz[2];
	}

	public void render(float[] vpsMatrix) {
		if(subgraph == null) return;

		System.arraycopy(vpsMatrix, 0, subgraphMatrix, 0, 16);
		Matrix.translateM(subgraphMatrix, 0, x, y, z);

		subgraph.render(subgraphMatrix);
	}
}
