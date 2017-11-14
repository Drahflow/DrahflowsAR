package name.drahflow.ar.geometry;

import android.opengl.Matrix;

public class Scaling implements Geometry {
	private float x, y, z;

	private Geometry subgraph;
	private float[] subgraphMatrix = new float[16];

	public Scaling() {
	}

	public Scaling(float x, float y, float z, Geometry subgraph) {
		setSubgraph(subgraph);
		setScaling(x, y, z);
	}

	public void setSubgraph(Geometry g) {
		subgraph = g;
	}

	public void setScaling(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public void render(float[] vpsMatrix) {
		if(subgraph == null) return;

		System.arraycopy(vpsMatrix, 0, subgraphMatrix, 0, 16);
		Matrix.scaleM(subgraphMatrix, 0, x, y, z);

		subgraph.render(subgraphMatrix);
	}

	@Override
	public Geometry onPointer(PointerEvent e) {
		if(subgraph == null) return null;

		try {
			e.x *= x;
			e.y *= y;
			e.z *= z;

			return subgraph.onPointer(e);
		} finally {
			e.x /= x;
			e.y /= y;
			e.z /= z;
		}
	}
}
