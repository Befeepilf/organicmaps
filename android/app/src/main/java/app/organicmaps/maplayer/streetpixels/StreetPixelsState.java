package app.organicmaps.maplayer.streetpixels;

import android.content.Context;
import android.view.View;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

public class StreetPixelsState
{
  public enum Status
  {
    NOT_READY,
    LOADING,
    READY
  }

  private final boolean mEnabled;
  @NonNull
  private final Status mStatus;

  public StreetPixelsState(boolean enabled, @NonNull Status status)
  {
    mEnabled = enabled;
    mStatus = status;
  }

  public boolean isEnabled()
  {
    return mEnabled;
  }

  @NonNull
  public Status getStatus()
  {
    return mStatus;
  }

  public void activate(@NonNull Context context, @Nullable View viewAbove, @Nullable View view)
  {
    /* Do nothing by default */
  }
}
