package app.organicmaps.settings;

public class TrailRoutingSettings
{
  public boolean m_preferTrails;
  public double m_trailPreference;

  public TrailRoutingSettings(boolean preferTrails, double trailPreference)
  {
    this.m_preferTrails = preferTrails;
    this.m_trailPreference = trailPreference;
  }

  public static TrailRoutingSettings LoadFromSettings()
  {
    boolean preferTrails = nativeGetPreferTrails();
    double trailPreference = nativeGetTrailPreference();
    return new TrailRoutingSettings(preferTrails, trailPreference);
  }

  public static void SaveToSettings(TrailRoutingSettings settings)
  {
    nativeSetPreferTrails(settings.m_preferTrails);
    nativeSetTrailPreference(settings.m_trailPreference);
  }

  private static native boolean nativeGetPreferTrails();
  private static native void nativeSetPreferTrails(boolean preferTrails);
  private static native double nativeGetTrailPreference();
  private static native void nativeSetTrailPreference(double trailPreference);
}
