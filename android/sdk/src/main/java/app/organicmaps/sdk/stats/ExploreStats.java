package app.organicmaps.sdk.stats;

import androidx.annotation.Keep;
import androidx.annotation.Nullable;

public class ExploreStats
{
  @Keep
  @SuppressWarnings("unused")
  public static class StatWeek
  {
    public long weekStartSec;
    public long exploredPixels;

    public StatWeek(long weekStartSec, long exploredPixels)
    {
      this.weekStartSec = weekStartSec;
      this.exploredPixels = exploredPixels;
    }
  }

  @Keep
  @SuppressWarnings("unused")
  public static class Region
  {
    public String id;
    public String name;

    public Region(String id, String name)
    {
      this.id = id;
      this.name = name;
    }
  }

  @Keep
  @SuppressWarnings("unused")
  public static class Payload
  {
    public StatWeek[] weeks;
    public Region[] regions;

    public Payload(StatWeek[] weeks, Region[] regions)
    {
      this.weeks = weeks;
      this.regions = regions;
    }
  }

  public static native Payload nativeGetAll();
  public static native Payload nativeGetForRegion(@Nullable String regionId);
}