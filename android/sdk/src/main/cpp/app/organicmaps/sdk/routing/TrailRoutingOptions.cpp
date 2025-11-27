#include <jni.h>
#include "app/organicmaps/sdk/Framework.hpp"
#include "app/organicmaps/sdk/core/jni_helper.hpp"
#include "routing/trail_routing_options.hpp"

extern "C"
{
JNIEXPORT jboolean JNICALL Java_app_organicmaps_sdk_routing_TrailRoutingOptions_nativeGetPreferTrails(JNIEnv *, jclass)
{
  routing::TrailRoutingOptions options = routing::TrailRoutingOptions::LoadFromSettings();
  return static_cast<jboolean>(options.m_preferTrails);
}

JNIEXPORT void JNICALL Java_app_organicmaps_sdk_routing_TrailRoutingOptions_nativeSetPreferTrails(JNIEnv *, jclass,
                                                                                                jboolean preferTrails)
{
  routing::TrailRoutingOptions options = routing::TrailRoutingOptions::LoadFromSettings();
  settings.m_preferTrails = static_cast<bool>(preferTrails);
  routing::TrailRoutingOptions::SaveToSettings(options);
}

JNIEXPORT jdouble JNICALL Java_app_organicmaps_sdk_routing_TrailRoutingOptions_nativeGetTrailPreference(JNIEnv *, jclass)
{
  routing::TrailRoutingOptions options = routing::TrailRoutingOptions::LoadFromSettings();
  return static_cast<jdouble>(options.m_trailPreference);
}

JNIEXPORT void JNICALL
Java_app_organicmaps_sdk_routing_TrailRoutingOptions_nativeSetTrailPreference(JNIEnv *, jclass, jdouble trailPreference)
{
  routing::TrailRoutingOptions options = routing::TrailRoutingOptions::LoadFromSettings();
  settings.m_trailPreference = static_cast<double>(trailPreference);
  routing::TrailRoutingOptions::SaveToSettings(options);
}
}
