package app.organicmaps.maplayer.streetpixels;

import android.app.Application;
import android.content.Context;

import androidx.annotation.NonNull;

import app.organicmaps.Framework;
import app.organicmaps.MwmApplication;

public class StreetPixelsManager
{
  @NonNull
  private final OnStreetPixelsChangedListener mListener;

  public StreetPixelsManager(@NonNull Application application)
  {
    mListener = new OnStreetPixelsChangedListener(application);
  }

  static public boolean isEnabled()
  {
    return Framework.nativeIsStreetPixelsLayerEnabled();
  }

  private void registerListener()
  {
    nativeAddListener(mListener);
  }

  static public void setEnabled(boolean isEnabled)
  {
    if (isEnabled == isEnabled())
      return;

    Framework.nativeSetStreetPixelsLayerEnabled(isEnabled);
  }

  public void initialize()
  {
    registerListener();
  }

  @NonNull
  public static StreetPixelsManager from(@NonNull Context context)
  {
    MwmApplication app = (MwmApplication) context.getApplicationContext();
    return app.getStreetPixelsManager();
  }

  private static native void nativeAddListener(@NonNull OnStreetPixelsChangedListener listener);
  private static native void nativeRemoveListener(@NonNull OnStreetPixelsChangedListener listener);
  private static native boolean nativeShouldShowNotification();

  public void attach(@NonNull StreetPixelsErrorDialogListener listener)
  {
    mListener.attach(listener);
  }

  public void detach()
  {
    mListener.detach();
  }

  public boolean shouldShowNotification()
  {
    return nativeShouldShowNotification();
  }
}
