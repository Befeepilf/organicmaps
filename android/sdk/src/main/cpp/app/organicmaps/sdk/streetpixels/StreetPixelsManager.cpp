#include <jni.h>
#include "app/organicmaps/sdk/Framework.hpp"
#include "app/organicmaps/sdk/core/jni_helper.hpp"
#include "app/organicmaps/sdk/platform/AndroidPlatform.hpp"

extern "C"
{
static void StreetPixelsStateChanged(bool enabled, StreetPixelsManager::StreetPixelsStatus status,
                                     std::string countryId, std::shared_ptr<jobject> const & listener)
{
  JNIEnv * env = jni::GetEnv();
  env->CallVoidMethod(*listener, jni::GetMethodID(env, *listener, "onStateChanged", "(ZILjava/lang/String;)V"),
                      static_cast<jboolean>(enabled), static_cast<jint>(status), jni::ToJavaString(env, countryId));
}

JNIEXPORT void JNICALL Java_app_organicmaps_sdk_maplayer_streetpixels_StreetPixelsManager_nativeAddListener(
  JNIEnv * env, jclass clazz, jobject listener)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  auto globalListener = jni::make_global_ref(listener);
  g_framework->SetStreetPixelsListener(
    [globalListener](bool enabled, StreetPixelsManager::StreetPixelsStatus status, std::string const & countryId)
    {
        StreetPixelsStateChanged(enabled, status, countryId, globalListener);
    }
  );
}

JNIEXPORT void JNICALL Java_app_organicmaps_sdk_maplayer_streetpixels_StreetPixelsManager_nativeRemoveListener(JNIEnv * env,
                                                                                                           jclass clazz)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  g_framework->SetStreetPixelsListener(nullptr);
}

JNIEXPORT jboolean JNICALL
Java_app_organicmaps_sdk_maplayer_streetpixels_StreetPixelsManager_nativeShouldShowNotification(JNIEnv * env, jclass clazz)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  auto const & manager = g_framework->NativeFramework()->GetStreetPixelsManager();
  auto const enabled = manager.GetState().enabled;
  return static_cast<jboolean>(enabled);
}

JNIEXPORT jdouble JNICALL Java_app_organicmaps_sdk_maplayer_streetpixels_StreetPixelsManager_nativeGetTrackExploredFraction(
  JNIEnv * env, jclass clazz, jlong trackId)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  auto const & manager = g_framework->NativeFramework()->GetStreetPixelsManager();
  double frac = manager.GetExploredFraction(static_cast<kml::TrackId>(trackId));
  return static_cast<jdouble>(frac);
}

JNIEXPORT jdouble JNICALL Java_app_organicmaps_sdk_maplayer_streetpixels_StreetPixelsManager_nativeGetTotalExploredFraction(
  JNIEnv * env, jclass clazz)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  auto const & manager = g_framework->NativeFramework()->GetStreetPixelsManager();
  double frac = manager.GetTotalExploredFraction();
  return static_cast<jdouble>(frac);
}
}