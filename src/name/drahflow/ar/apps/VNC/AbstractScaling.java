/**
 * Copyright (C) 2009 Michael A. MacDonald
 */
package name.drahflow.ar.apps.VNC;

import android.widget.ImageView;

/**
 * @author Michael A. MacDonald
 * 
 * A scaling mode for the VncCanvas; based on ImageView.ScaleType
 */
abstract class AbstractScaling {
	static final int FIT_TO_SCREEN = 1;
	static final int ONE_TO_ONE = 2;
	static final int ZOOMABLE = 3;

	private static final int scaleModeIds[] = { FIT_TO_SCREEN, ONE_TO_ONE, ZOOMABLE };
	
	private static AbstractScaling[] scalings;

	static AbstractScaling getById(int id)
	{
		if ( scalings==null)
		{
			scalings=new AbstractScaling[scaleModeIds.length];
		}
		for ( int i=0; i<scaleModeIds.length; ++i)
		{
			if ( scaleModeIds[i]==id)
			{
				if ( scalings[i]==null)
				{
					switch ( id )
					{
					case FIT_TO_SCREEN:
						scalings[i]=new FitToScreenScaling();
						break;
					case ONE_TO_ONE:
						scalings[i]=new OneToOneScaling();
						break;
					case ZOOMABLE :
						scalings[i]=new ZoomScaling();
						break;
					}
				}
				return scalings[i];
			}
		}
		throw new IllegalArgumentException("Unknown scaling id " + id);
	}
	
	float getScale() { return 1; }
	
	void zoomIn(Window activity) {}
	void zoomOut(Window activity) {}
	
	static AbstractScaling getByScaleType(ImageView.ScaleType scaleType)
	{
		for (int i : scaleModeIds)
		{
			AbstractScaling s = getById(i);
			if (s.scaleType==scaleType)
				return s;
		}
		throw new IllegalArgumentException("Unsupported scale type: "+ scaleType.toString());
	}
	
	private int id;
	protected ImageView.ScaleType scaleType;
	
	protected AbstractScaling(int id, ImageView.ScaleType scaleType)
	{
		this.id = id;
		this.scaleType = scaleType;
	}
	
	/**
	 * 
	 * @return Id corresponding to menu item that sets this scale type
	 */
	int getId()
	{
		return id;
	}

	/**
	 * Sets the activity's scale type to the scaling
	 * @param activity
	 */
	void setScaleTypeForActivity(Window activity)
	{
		// activity.zoomer.hide();
		activity.vncCanvas.scaling = this;
		activity.vncCanvas.setScaleType(scaleType);
		// activity.getConnection().setScaleMode(scaleType);

		// if (activity.inputHandler == null || ! isValidInputMode(activity.getModeIdFromHandler(activity.inputHandler))) {
		// 	activity.inputHandler=activity.getInputHandlerById(getDefaultHandlerId());
		// 	// activity.getConnection().setInputMode(activity.inputHandler.getName());
		// }
		// activity.getConnection().Gen_update(activity.database.getWritableDatabase());
		// activity.updateInputMenu();
	}
	
	/**
	 * True if this scale type allows panning of the image
	 * @return
	 */
	abstract boolean isAbleToPan();
	
	/**
	 * Change the scaling and focus dynamically, as from a detected scale gesture
	 * @param activity Activity containing to canvas to scale
	 * @param scaleFactor Factor by which to adjust scaling
	 * @param fx Focus X of center of scale change
	 * @param fy Focus Y of center of scale change
	 */
	void adjust(Window activity, float scaleFactor, float fx, float fy)
	{
	
	}
}
