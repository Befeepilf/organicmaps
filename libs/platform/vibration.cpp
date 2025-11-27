#include "platform/vibration.hpp"
#if defined(OMIM_OS_ANDROID)
#include <jni.h>
#include "android/sdk/src/main/cpp/app/organicmaps/sdk/core/jni_helper.hpp"

namespace platform
{
void Vibrate(uint32_t durationMs)
{
  JNIEnv * env = jni::GetEnv();
  if (env == nullptr)
    return;

  static jmethodID const vibrateId = jni::GetStaticMethodID(env, g_utilsClazz, "vibrate", "(J)V");
  if (vibrateId == nullptr)
    return;

  env->CallStaticVoidMethod(g_utilsClazz, vibrateId, static_cast<jlong>(durationMs));
}

void VibratePattern(uint32_t const * durations, uint32_t const * delays, size_t count)
{
  JNIEnv * env = jni::GetEnv();
  if (env == nullptr)
    return;

  static jmethodID const vibratePatternId = jni::GetStaticMethodID(env, g_utilsClazz, "vibratePattern", "([J[J)V");
  if (vibratePatternId == nullptr)
    return;

  jlongArray durationsArray = env->NewLongArray(static_cast<jsize>(count));
  jlongArray delaysArray = env->NewLongArray(static_cast<jsize>(count));

  if (durationsArray == nullptr || delaysArray == nullptr)
    return;

  std::vector<jlong> durationsVec(durations, durations + count);
  std::vector<jlong> delaysVec(delays, delays + count);

  env->SetLongArrayRegion(durationsArray, 0, static_cast<jsize>(count), durationsVec.data());
  env->SetLongArrayRegion(delaysArray, 0, static_cast<jsize>(count), delaysVec.data());

  env->CallStaticVoidMethod(g_utilsClazz, vibratePatternId, durationsArray, delaysArray);

  env->DeleteLocalRef(durationsArray);
  env->DeleteLocalRef(delaysArray);
}
}  // namespace platform
#else

namespace platform
{
void Vibrate(uint32_t /*durationMs*/)
{
  // Vibration is not supported on this platform.
}

void VibratePattern(uint32_t const * /*durations*/, uint32_t const * /*delays*/, size_t /*count*/)
{
  // Vibration is not supported on this platform.
}
}  // namespace platform

#endif  // OMIM_OS_ANDROID