package app.organicmaps.maplayer.streetpixels;

import android.content.Context;
import android.view.View;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import app.organicmaps.R;
import app.organicmaps.util.Utils;

public enum StreetPixelsState
{
  DISABLED,
  ENABLED,
  NODATA;

  public void activate(@NonNull Context context, @Nullable View viewAbove, @Nullable View view)
  {
    /* Do nothing by default */
  }
}
