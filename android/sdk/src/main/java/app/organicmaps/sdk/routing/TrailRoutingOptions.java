package app.organicmaps.sdk.routing;

public class TrailRoutingOptions
{
  public boolean m_preferTrails;
  public double m_trailPreference;

  public TrailRoutingOptions(boolean preferTrails, double trailPreference)
  {
    this.m_preferTrails = preferTrails;
    this.m_trailPreference = trailPreference;
  }

  public static TrailRoutingOptions LoadFromSettings()
  {
    boolean preferTrails = nativeGetPreferTrails();
    double trailPreference = nativeGetTrailPreference();
    return new TrailRoutingOptions(preferTrails, trailPreference);
  }

  public static void SaveToSettings(TrailRoutingOptions settings)
  {
    nativeSetPreferTrails(settings.m_preferTrails);
    nativeSetTrailPreference(settings.m_trailPreference);
  }

  private static native boolean nativeGetPreferTrails();
  private static native void nativeSetPreferTrails(boolean preferTrails);
  private static native double nativeGetTrailPreference();
  private static native void nativeSetTrailPreference(double trailPreference);
}