package app.organicmaps.sdk.maplayer.streetpixels;

import android.app.Application;
import android.content.Context;

import androidx.annotation.NonNull;

import app.organicmaps.sdk.Framework;

public class StreetPixelsManager
{
  private static volatile StreetPixelsState.Status sStatus = StreetPixelsState.Status.NOT_READY;

  @NonNull
  private final OnStreetPixelsChangedListener mListener;

  public interface Callback
  {
    void onStateChanged(boolean enabled, @NonNull StreetPixelsState.Status status, @NonNull String countryId);
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

  public StreetPixelsManager()
  {
    mListener = new OnStreetPixelsChangedListener();
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

  private static native void nativeAddListener(@NonNull OnStreetPixelsChangedListener listener);
  private static native void nativeRemoveListener(@NonNull OnStreetPixelsChangedListener listener);
  private static native boolean nativeShouldShowNotification();
  private static native double nativeGetTrackExploredFraction(long trackId);
  private static native double nativeGetTotalExploredFraction();

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

  public double getTrackExploredFraction(long trackId)
  {
    return nativeGetTrackExploredFraction(trackId);
  }

  public double getTotalExploredFraction()
  {
    return nativeGetTotalExploredFraction();
  }

  public static boolean isLoading()
  {
    return sStatus == StreetPixelsState.Status.LOADING;
  }

  public static void updateState(@NonNull StreetPixelsState state)
  {
    sStatus = state.getStatus();
    java.util.List<Callback> snapshot;
    synchronized (sCallbacks)
    {
      snapshot = new java.util.ArrayList<>(sCallbacks);
    }
    for (Callback cb : snapshot)
    {
      cb.onStateChanged(state.isEnabled(), state.getStatus(), state.getCountryId());
    }
  }
}