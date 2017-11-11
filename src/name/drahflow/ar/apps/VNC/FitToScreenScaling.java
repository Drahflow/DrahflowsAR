/**
 * Copyright (C) 2009 Michael A. MacDonald
 */
package name.drahflow.ar.apps.VNC;

import android.widget.ImageView.ScaleType;

/**
 * @author Michael A. MacDonald
 */
class FitToScreenScaling extends AbstractScaling {

	/**
	 * @param id
	 * @param scaleType
	 */
	FitToScreenScaling() {
		super(FIT_TO_SCREEN, ScaleType.FIT_CENTER);
	}

	/* (non-Javadoc)
	 * @see name.drahflow.ar.apps.VNC.AbstractScaling#isAbleToPan()
	 */
	@Override
	boolean isAbleToPan() {
		return false;
	}

	/* (non-Javadoc)
	 * @see name.drahflow.ar.apps.VNC.AbstractScaling#setCanvasScaleType(name.drahflow.ar.apps.VNC.VncCanvas)
	 */
	@Override
	void setScaleTypeForActivity(Window activity) {
		super.setScaleTypeForActivity(activity);
		activity.vncCanvas.absoluteXPosition = activity.vncCanvas.absoluteYPosition = 0;
		activity.vncCanvas.scrollTo(0, 0);
	}

}
