package app.organicmaps.sdk.maplayer.streetpixels;

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
  @NonNull
  private final String mCountryId;

  public StreetPixelsState(boolean enabled, @NonNull Status status, @NonNull String countryId)
  {
    mEnabled = enabled;
    mStatus = status;
    mCountryId = countryId;
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

  @NonNull
  public String getCountryId()
  {
    return mCountryId;
  }
}