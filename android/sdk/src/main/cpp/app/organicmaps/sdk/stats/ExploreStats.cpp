#include "app/organicmaps/sdk/Framework.hpp"
#include "app/organicmaps/sdk/core/jni_helper.hpp"

#include "map/framework.hpp"

namespace
{
jclass g_PayloadClazz;
jmethodID g_PayloadCtor;

jclass g_StatWeekClazz;
jmethodID g_StatWeekCtor;

jclass g_RegionClazz;
jmethodID g_RegionCtor;

void EnsureIds(JNIEnv * env)
{
  if (g_PayloadClazz)
    return;
  g_PayloadClazz = jni::GetGlobalClassRef(env, "app/organicmaps/sdk/stats/ExploreStats$Payload");
  g_PayloadCtor = jni::GetConstructorID(
    env, g_PayloadClazz,
    "([Lapp/organicmaps/sdk/stats/ExploreStats$StatWeek;[Lapp/organicmaps/sdk/stats/ExploreStats$Region;)V");
  g_StatWeekClazz = jni::GetGlobalClassRef(env, "app/organicmaps/sdk/stats/ExploreStats$StatWeek");
  g_StatWeekCtor = jni::GetConstructorID(env, g_StatWeekClazz, "(JJ)V");
  g_RegionClazz = jni::GetGlobalClassRef(env, "app/organicmaps/sdk/stats/ExploreStats$Region");
  g_RegionCtor = jni::GetConstructorID(env, g_RegionClazz, "(Ljava/lang/String;Ljava/lang/String;)V");
}

jobject MakePayload(JNIEnv * env, std::vector<std::pair<uint64_t, uint64_t>> const & weeks,
                    std::vector<std::pair<std::string, std::string>> const & regions)
{
  EnsureIds(env);
  jobjectArray jweeks = env->NewObjectArray(static_cast<jsize>(weeks.size()), g_StatWeekClazz, nullptr);
  for (jsize i = 0; i < static_cast<jsize>(weeks.size()); ++i)
  {
    auto const & w = weeks[static_cast<size_t>(i)];
    jobject jw =
      env->NewObject(g_StatWeekClazz, g_StatWeekCtor, static_cast<jlong>(w.first), static_cast<jlong>(w.second));
    env->SetObjectArrayElement(jweeks, i, jw);
  }

  jobjectArray jregions = env->NewObjectArray(static_cast<jsize>(regions.size()), g_RegionClazz, nullptr);
  for (jsize i = 0; i < static_cast<jsize>(regions.size()); ++i)
  {
    auto const & r = regions[static_cast<size_t>(i)];
    jni::ScopedLocalRef<jstring> id(env, jni::ToJavaString(env, r.first));
    jni::ScopedLocalRef<jstring> name(env, jni::ToJavaString(env, r.second));
    jobject jr = env->NewObject(g_RegionClazz, g_RegionCtor, id.get(), name.get());
    env->SetObjectArrayElement(jregions, i, jr);
  }

  return env->NewObject(g_PayloadClazz, g_PayloadCtor, jweeks, jregions);
}
}  // namespace

extern "C"
{
JNIEXPORT jobject JNICALL Java_app_organicmaps_sdk_stats_ExploreStats_nativeGetAll(JNIEnv * env, jclass)
{
  std::vector<std::pair<uint64_t, uint64_t>> weeks;
  std::vector<std::pair<std::string, std::string>> regions;
  frm()->GetExploreStatsAggregatedWeeks(weeks, "");
  frm()->GetExploreStatsRegions(regions);
  return MakePayload(env, weeks, regions);
}

JNIEXPORT jobject JNICALL Java_app_organicmaps_sdk_stats_ExploreStats_nativeGetForRegion(JNIEnv * env, jclass,
                                                                                     jstring regionId)
{
  std::string id;
  if (regionId)
    id = jni::ToNativeString(env, regionId);
  std::vector<std::pair<uint64_t, uint64_t>> weeks;
  std::vector<std::pair<std::string, std::string>> regions;
  frm()->GetExploreStatsAggregatedWeeks(weeks, id);
  frm()->GetExploreStatsRegions(regions);
  return MakePayload(env, weeks, regions);
}
}
