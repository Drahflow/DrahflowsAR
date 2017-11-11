/**
 * Copyright (C) 2009 Michael A. MacDonald
 */
package name.drahflow.ar.apps.VNC;

import android.graphics.Matrix;
import android.util.Log;
import android.widget.ImageView.ScaleType;

/**
 * @author Michael A. MacDonald
 */
class ZoomScaling extends AbstractScaling {
	
	static final String TAG = "ZoomScaling";

	private Matrix matrix;
	int canvasXOffset;
	int canvasYOffset;
	float scaling;
	float minimumScale;
	
	/**
	 * @param id
	 * @param scaleType
	 */
	public ZoomScaling() {
		super(ZOOMABLE, ScaleType.MATRIX);
		matrix = new Matrix();
		scaling = 1;
	}

	/* (non-Javadoc)
	 * @see name.drahflow.ar.apps.VNC.AbstractScaling#isAbleToPan()
	 */
	@Override
	boolean isAbleToPan() {
		return true;
	}

	/**
	 * Call after scaling and matrix have been changed to resolve scrolling
	 * @param activity
	 */
	private void resolveZoom(Window activity)
	{
		activity.vncCanvas.scrollToAbsolute();
		activity.vncCanvas.pan(0,0);
	}
	
	/* (non-Javadoc)
	 * @see name.drahflow.ar.apps.VNC.AbstractScaling#zoomIn(name.drahflow.ar.apps.VNC.VncCanvasActivity)
	 */
	@Override
	void zoomIn(Window activity) {
		resetMatrix();
		standardizeScaling();
		scaling += 0.25;
		if (scaling > 4.0)
		{
			scaling = (float)4.0;
			// activity.zoomer.setIsZoomInEnabled(false);
		}
		// activity.zoomer.setIsZoomOutEnabled(true);
		matrix.postScale(scaling, scaling);
		//Log.v(TAG,String.format("before set matrix scrollx = %d scrolly = %d", activity.vncCanvas.getScrollX(), activity.vncCanvas.getScrollY()));
		activity.vncCanvas.setImageMatrix(matrix);
		resolveZoom(activity);
	}

	/* (non-Javadoc)
	 * @see name.drahflow.ar.apps.VNC.AbstractScaling#getScale()
	 */
	@Override
	float getScale() {
		return scaling;
	}

	/* (non-Javadoc)
	 * @see name.drahflow.ar.apps.VNC.AbstractScaling#zoomOut(name.drahflow.ar.apps.VNC.VncCanvasActivity)
	 */
	@Override
	void zoomOut(Window activity) {
		resetMatrix();
		standardizeScaling();
		scaling -= 0.25;
		if (scaling < minimumScale)
		{
			scaling = minimumScale;
			// activity.zoomer.setIsZoomOutEnabled(false);
		}
		// activity.zoomer.setIsZoomInEnabled(true);
		matrix.postScale(scaling, scaling);
		//Log.v(TAG,String.format("before set matrix scrollx = %d scrolly = %d", activity.vncCanvas.getScrollX(), activity.vncCanvas.getScrollY()));
		activity.vncCanvas.setImageMatrix(matrix);
		//Log.v(TAG,String.format("after set matrix scrollx = %d scrolly = %d", activity.vncCanvas.getScrollX(), activity.vncCanvas.getScrollY()));
		resolveZoom(activity);
	}

	/* (non-Javadoc)
	 * @see name.drahflow.ar.apps.VNC.AbstractScaling#adjust(name.drahflow.ar.apps.VNC.VncCanvasActivity, float, float, float)
	 */
	@Override
	void adjust(Window activity, float scaleFactor, float fx,
			float fy) {
		float newScale = scaleFactor * scaling;
		if (scaleFactor < 1)
		{
			if (newScale < minimumScale)
			{
				newScale = minimumScale;
				// activity.zoomer.setIsZoomOutEnabled(false);
			}
			// activity.zoomer.setIsZoomInEnabled(true);
		}
		else
		{
			if (newScale > 4)
			{
				newScale = 4;
				// activity.zoomer.setIsZoomInEnabled(false);
			}
			// activity.zoomer.setIsZoomOutEnabled(true);
		}
		// ax is the absolute x of the focus
		int xPan = activity.vncCanvas.absoluteXPosition;
		float ax = (fx / scaling) + xPan;
		float newXPan = (scaling * xPan - scaling * ax + newScale * ax)/newScale;
		int yPan = activity.vncCanvas.absoluteYPosition;
		float ay = (fy / scaling) + yPan;
		float newYPan = (scaling * yPan - scaling * ay + newScale * ay)/newScale;
		resetMatrix();
		scaling = newScale;
		matrix.postScale(scaling, scaling);
		activity.vncCanvas.setImageMatrix(matrix);
		resolveZoom(activity);
		activity.vncCanvas.pan((int)(newXPan - xPan), (int)(newYPan - yPan));
	}

	private void resetMatrix()
	{
		matrix.reset();
		matrix.preTranslate(canvasXOffset, canvasYOffset);
	}
	
	/**
	 *  Set scaling to one of the clicks on the zoom scale
	 */
	private void standardizeScaling()
	{
		scaling = ((float)((int)(scaling * 4))) / 4;
	}

	/* (non-Javadoc)
	 * @see name.drahflow.ar.apps.VNC.AbstractScaling#setScaleTypeForActivity(name.drahflow.ar.apps.VNC.VncCanvasActivity)
	 */
	@Override
	void setScaleTypeForActivity(Window activity) {
		super.setScaleTypeForActivity(activity);
		scaling = (float)1.0;
		minimumScale = activity.vncCanvas.bitmapData.getMinimumScale();
		canvasXOffset = -activity.vncCanvas.getCenteredXOffset();
		canvasYOffset = -activity.vncCanvas.getCenteredYOffset();
		resetMatrix();
		activity.vncCanvas.setImageMatrix(matrix);
		// Reset the pan position to (0,0)
		resolveZoom(activity);
	}

}
