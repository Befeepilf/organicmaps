#include <jni.h>
#include <android/app/src/main/cpp/app/organicmaps/Framework.hpp>
#include <android/app/src/main/cpp/app/organicmaps/core/jni_helper.hpp>
#include "routing/routing_options.hpp"
#include "routing/routing_settings.hpp"

routing::RoutingOptions::Road makeValue(jint option)
{
  auto const road = static_cast<uint8_t>(1u << static_cast<int>(option));
  CHECK_LESS(road, static_cast<uint8_t>(routing::RoutingOptions::Road::Max), ());
  return static_cast<routing::RoutingOptions::Road>(road);
}

extern "C"
{
JNIEXPORT jboolean JNICALL Java_app_organicmaps_sdk_routing_RoutingOptions_nativeHasOption(JNIEnv *, jclass,
                                                                                           jint option)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  routing::RoutingOptions routingOptions = routing::RoutingOptions::LoadCarOptionsFromSettings();
  routing::RoutingOptions::Road road = makeValue(option);
  return static_cast<jboolean>(routingOptions.Has(road));
}

JNIEXPORT void JNICALL Java_app_organicmaps_sdk_routing_RoutingOptions_nativeAddOption(JNIEnv *, jclass, jint option)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  routing::RoutingOptions routingOptions = routing::RoutingOptions::LoadCarOptionsFromSettings();
  routing::RoutingOptions::Road road = makeValue(option);
  routingOptions.Add(road);
  routing::RoutingOptions::SaveCarOptionsToSettings(routingOptions);
}

JNIEXPORT void JNICALL Java_app_organicmaps_sdk_routing_RoutingOptions_nativeRemoveOption(JNIEnv *, jclass, jint option)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  routing::RoutingOptions routingOptions = routing::RoutingOptions::LoadCarOptionsFromSettings();
  routing::RoutingOptions::Road road = makeValue(option);
  routingOptions.Remove(road);
  routing::RoutingOptions::SaveCarOptionsToSettings(routingOptions);
}

JNIEXPORT jboolean JNICALL Java_app_organicmaps_settings_TrailRoutingSettings_nativeGetPreferTrails(JNIEnv *, jclass)
{
  routing::TrailRoutingSettings settings = routing::TrailRoutingSettings::LoadFromSettings();
  return static_cast<jboolean>(settings.m_preferTrails);
}

JNIEXPORT void JNICALL Java_app_organicmaps_settings_TrailRoutingSettings_nativeSetPreferTrails(JNIEnv *, jclass,
                                                                                                jboolean preferTrails)
{
  routing::TrailRoutingSettings settings = routing::TrailRoutingSettings::LoadFromSettings();
  settings.m_preferTrails = static_cast<bool>(preferTrails);
  routing::TrailRoutingSettings::SaveToSettings(settings);
}

JNIEXPORT jdouble JNICALL Java_app_organicmaps_settings_TrailRoutingSettings_nativeGetTrailPreference(JNIEnv *, jclass)
{
  routing::TrailRoutingSettings settings = routing::TrailRoutingSettings::LoadFromSettings();
  return static_cast<jdouble>(settings.m_trailPreference);
}

JNIEXPORT void JNICALL
Java_app_organicmaps_settings_TrailRoutingSettings_nativeSetTrailPreference(JNIEnv *, jclass, jdouble trailPreference)
{
  routing::TrailRoutingSettings settings = routing::TrailRoutingSettings::LoadFromSettings();
  settings.m_trailPreference = static_cast<double>(trailPreference);
  routing::TrailRoutingSettings::SaveToSettings(settings);
}
}
