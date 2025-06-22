#include <jni.h>
#include "app/organicmaps/Framework.hpp"
#include "app/organicmaps/core/jni_helper.hpp"
#include "app/organicmaps/platform/AndroidPlatform.hpp"

using namespace std::placeholders;

extern "C"
{
static void StreetPixelsStateChanged(bool enabled, StreetPixelsManager::StreetPixelsStatus status,
                                     std::shared_ptr<jobject> const & listener)
{
  JNIEnv * env = jni::GetEnv();
  env->CallVoidMethod(*listener, jni::GetMethodID(env, *listener, "onStateChanged", "(ZI)V"),
                      static_cast<jboolean>(enabled), static_cast<jint>(status));
}

JNIEXPORT void JNICALL Java_app_organicmaps_maplayer_streetpixels_StreetPixelsManager_nativeAddListener(
  JNIEnv * env, jclass clazz, jobject listener)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  g_framework->SetStreetPixelsListener(
    std::bind(&StreetPixelsStateChanged, std::placeholders::_1, std::placeholders::_2, jni::make_global_ref(listener)));
}

JNIEXPORT void JNICALL Java_app_organicmaps_maplayer_streetpixels_StreetPixelsManager_nativeRemoveListener(JNIEnv * env,
                                                                                                           jclass clazz)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  g_framework->SetStreetPixelsListener(nullptr);
}

JNIEXPORT jboolean JNICALL
Java_app_organicmaps_maplayer_streetpixels_StreetPixelsManager_nativeShouldShowNotification(JNIEnv * env, jclass clazz)
{
  CHECK(g_framework, ("Framework isn't created yet!"));
  auto const & manager = g_framework->NativeFramework()->GetStreetPixelsManager();
  auto const enabled = manager.GetState().enabled;
  return static_cast<jboolean>(enabled);
}
}
