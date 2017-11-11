package name.drahflow.ar.geometry;

import android.opengl.Matrix;

public class Translation implements Geometry {
	private float x, y, z;

	private Geometry subgraph;
	private float[] subgraphMatrix = new float[16];

	public Translation() {
	}

	public Translation(Geometry subgraph, float x, float y, float z) {
		setSubgraph(subgraph);
		setTranslation(x, y, z);
	}

	public void setSubgraph(Geometry g) {
		subgraph = g;
	}

	public void setTranslation(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public void render(float[] vpsMatrix) {
		if(subgraph == null) return;

		System.arraycopy(vpsMatrix, 0, subgraphMatrix, 0, 16);
		Matrix.translateM(subgraphMatrix, 0, x, y, z);

		subgraph.render(subgraphMatrix);
	}

	@Override
	public Geometry onPointer(PointerEvent e) {
		if(subgraph == null) return null;

		try {
			e.x -= x;
			e.y -= y;
			e.z -= z;

			return subgraph.onPointer(e);
		} finally {
			e.x += x;
			e.y += y;
			e.z += z;
		}
	}
}
