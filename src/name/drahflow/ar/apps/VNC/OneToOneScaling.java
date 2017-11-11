/**
 * Copyright (C) 2009 Michael A. MacDonald
 */
package name.drahflow.ar.apps.VNC;

import android.widget.ImageView.ScaleType;

/**
 * @author Michael A. MacDonald
 */
class OneToOneScaling extends AbstractScaling {

	/**
	 * @param id
	 * @param scaleType
	 */
	public OneToOneScaling() {
		super(ONE_TO_ONE, ScaleType.CENTER);
	}

	/* (non-Javadoc)
	 * @see name.drahflow.ar.apps.VNC.AbstractScaling#isAbleToPan()
	 */
	@Override
	boolean isAbleToPan() {
		return true;
	}

	/* (non-Javadoc)
	 * @see name.drahflow.ar.apps.VNC.AbstractScaling#setScaleTypeForActivity(name.drahflow.ar.apps.VNC.VncCanvasActivity)
	 */
	@Override
	void setScaleTypeForActivity(Window activity) {
		super.setScaleTypeForActivity(activity);
		activity.vncCanvas.scrollToAbsolute();
		activity.vncCanvas.pan(0,0);
	}
}
