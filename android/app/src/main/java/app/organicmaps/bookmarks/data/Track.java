package app.organicmaps.bookmarks.data;

import androidx.annotation.Keep;

import app.organicmaps.util.Distance;

// Called from JNI.
@Keep
@SuppressWarnings("unused")
public class Track
{
  private final long mTrackId;
  private final long mCategoryId;
  private final String mName;
  private final Distance mLength;
  private final int mColor;
  private final double mExploredFraction;

  Track(long trackId, long categoryId, String name, Distance length, int color)
  {
    mTrackId = trackId;
    mCategoryId = categoryId;
    mName = name;
    mLength = length;
    mColor = color;
    mExploredFraction = 0.0;
  }

  public Track(long trackId, long categoryId, String name, Distance length, int color, double exploredFraction)
  {
    mTrackId = trackId;
    mCategoryId = categoryId;
    mName = name;
    mLength = length;
    mColor = color;
    mExploredFraction = exploredFraction;
  }

  public String getName() { return mName; }

  public Distance getLength() { return mLength;}

  public int getColor() { return mColor; }

  public long getTrackId() { return mTrackId; }

  public long getCategoryId() { return mCategoryId; }

  public double getExploredFraction() { return mExploredFraction; }

  public String getTrackDescription()
  {
    return BookmarkManager.INSTANCE.getTrackDescription(mTrackId);
  }
}
