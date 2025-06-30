package app.organicmaps.maplayer.streetpixels;

import android.app.Application;

import androidx.annotation.Keep;
import androidx.annotation.NonNull;

class OnStreetPixelsChangedListener
{
  @NonNull
  private final Application mApp;
  private StreetPixelsErrorDialogListener mListener;

  OnStreetPixelsChangedListener(@NonNull Application app)
  {
    mApp = app;
  }

  // Called from JNI.
  @Keep
  @SuppressWarnings("unused")
  public void onStateChanged(boolean enabled, int status, @NonNull String countryId)
  {
    StreetPixelsState.Status newStatus = StreetPixelsState.Status.values()[status];
    StreetPixelsState state = new StreetPixelsState(enabled, newStatus, countryId);
    StreetPixelsManager.updateState(state);
    if (mListener == null)
    {
      state.activate(mApp, null, null);
      return;
    }

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
