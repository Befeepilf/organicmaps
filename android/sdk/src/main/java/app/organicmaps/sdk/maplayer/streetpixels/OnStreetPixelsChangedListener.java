package app.organicmaps.sdk.maplayer.streetpixels;

import android.app.Application;

import androidx.annotation.Keep;
import androidx.annotation.NonNull;


class OnStreetPixelsChangedListener
{
  private StreetPixelsErrorDialogListener mListener;

  OnStreetPixelsChangedListener()
  {}

  // Called from JNI.
  @Keep
  @SuppressWarnings("unused")
  public void onStateChanged(boolean enabled, int status, @NonNull String countryId)
  {
    StreetPixelsState.Status newStatus = StreetPixelsState.Status.values()[status];
    StreetPixelsState state = new StreetPixelsState(enabled, newStatus, countryId);
    StreetPixelsManager.updateState(state);
    if (mListener == null)
      return;

    mListener.onStateChanged(state);
  }

  public void attach(@NonNull StreetPixelsErrorDialogListener listener)
  {
    mListener = listener;
  }

  public void detach()
  {
    mListener = null;
  }
}