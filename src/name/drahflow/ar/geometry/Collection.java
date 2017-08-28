package name.drahflow.ar.geometry;

import java.util.ArrayList;
import java.util.Arrays;

public class Collection implements Geometry {
	private ArrayList<Geometry> children = new ArrayList<>();

	public Collection(Geometry... g) {
		children.addAll(Arrays.asList(g));
	}

	public void render(float[] vpsMatrix) {
		for(Geometry g: children) {
			g.render(vpsMatrix);
		}
	}
}
