package name.drahflow.ar.geometry;

public interface Geometry {
	public void render(float[] vpsMatrix);

	public Geometry onPointer(PointerEvent e);
}
