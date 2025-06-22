package app.organicmaps.maplayer.streetpixels;

import android.app.Application;
import android.content.Context;
import android.util.Log;

import androidx.annotation.NonNull;

import app.organicmaps.Framework;
import app.organicmaps.MwmApplication;

public class StreetPixelsManager
{
  private static volatile StreetPixelsState.Status sStatus = StreetPixelsState.Status.NOT_READY;

  @NonNull
  private final OnStreetPixelsChangedListener mListener;

  public interface Callback
  {
    void onStateChanged(boolean enabled, @NonNull StreetPixelsState.Status status);
  }

  @NonNull
  private static final java.util.List<Callback> sCallbacks = new java.util.ArrayList<>();

  public static void registerCallback(@NonNull Callback callback)
  {
    synchronized (sCallbacks)
    {
      if (!sCallbacks.contains(callback))
        sCallbacks.add(callback);
    }
  }

  public static void unregisterCallback(@NonNull Callback callback)
  {
    synchronized (sCallbacks)
    {
      sCallbacks.remove(callback);
    }
  }

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

  public static boolean isLoading()
  {
    return sStatus == StreetPixelsState.Status.LOADING;
  }

  static void updateStatus(@NonNull StreetPixelsState.Status status)
  {
    sStatus = status;
    Log.i("StreetPixelsManager", "updateStatus: " + status.name());

    boolean enabled = isEnabled();
    java.util.List<Callback> snapshot;
    synchronized (sCallbacks)
    {
      snapshot = new java.util.ArrayList<>(sCallbacks);
    }
    for (Callback cb : snapshot)
    {
      cb.onStateChanged(enabled, status);
    }
  }
}
