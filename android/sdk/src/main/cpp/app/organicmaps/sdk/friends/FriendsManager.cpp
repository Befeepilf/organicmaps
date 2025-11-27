#include <jni.h>

#include "app/organicmaps/sdk/core/jni_helper.hpp"

#include "map/friends_manager.hpp"

static FriendsManager g_friends;

namespace
{
jclass g_FriendClazz = nullptr;
jmethodID g_FriendCtor = nullptr;

jclass g_FriendsPayloadClazz = nullptr;
jmethodID g_FriendsPayloadCtor = nullptr;

jobjectArray ToJavaFriendsArray(JNIEnv * env, std::vector<FriendRecord> const & v)
{
  if (!g_FriendClazz)
  {
    g_FriendClazz = jni::GetGlobalClassRef(env, "app/organicmaps/friends/Friends$Friend");
    g_FriendCtor = jni::GetConstructorID(env, g_FriendClazz, "(Ljava/lang/String;Ljava/lang/String;)V");
  }
  jobjectArray arr = env->NewObjectArray(static_cast<jsize>(v.size()), g_FriendClazz, nullptr);
  for (size_t i = 0; i < v.size(); ++i)
  {
    jni::TScopedLocalRef const uid(env, jni::ToJavaString(env, v[i].m_userId));
    jni::TScopedLocalRef const uname(env, jni::ToJavaString(env, v[i].m_username));
    jobject obj = env->NewObject(g_FriendClazz, g_FriendCtor, uid.get(), uname.get());
    env->SetObjectArrayElement(arr, static_cast<jsize>(i), obj);
    env->DeleteLocalRef(obj);
  }
  return arr;
}
}  // namespace

extern "C"
{
JNIEXPORT jobject JNICALL Java_app_organicmaps_sdk_friends_Friends_nativeGetLists(JNIEnv * env, jclass)
{
  (void)g_friends.EnsureCacheLoaded();
  if (!g_FriendsPayloadClazz)
  {
    g_FriendsPayloadClazz = jni::GetGlobalClassRef(env, "app/organicmaps/friends/Friends$FriendsPayload");
    g_FriendsPayloadCtor = jni::GetConstructorID(env, g_FriendsPayloadClazz,
                                                 "([Lapp/organicmaps/friends/Friends$Friend;[Lapp/organicmaps/friends/"
                                                 "Friends$Friend;[Lapp/organicmaps/friends/Friends$Friend;)V");
  }
  auto const & lists = g_friends.GetLists();
  jobjectArray accepted = ToJavaFriendsArray(env, lists.m_accepted);
  jobjectArray incoming = ToJavaFriendsArray(env, lists.m_incoming);
  jobjectArray outgoing = ToJavaFriendsArray(env, lists.m_outgoing);
  return env->NewObject(g_FriendsPayloadClazz, g_FriendsPayloadCtor, accepted, incoming, outgoing);
}

JNIEXPORT void JNICALL Java_app_organicmaps_sdk_friends_Friends_nativeRefresh(JNIEnv *, jclass) { g_friends.Refresh(); }

JNIEXPORT jobjectArray JNICALL Java_app_organicmaps_sdk_friends_Friends_nativeSearchByUsername(JNIEnv * env, jclass,
                                                                                           jstring query)
{
  auto const q = jni::ToNativeString(env, query);
  auto const results = g_friends.SearchByUsername(q);
  return ToJavaFriendsArray(env, results);
}

JNIEXPORT jboolean JNICALL Java_app_organicmaps_sdk_friends_Friends_nativeSendRequest(JNIEnv * env, jclass, jstring userId)
{
  auto const id = jni::ToNativeString(env, userId);
  return static_cast<jboolean>(g_friends.SendRequest(id));
}

JNIEXPORT jboolean JNICALL Java_app_organicmaps_sdk_friends_Friends_nativeAcceptRequest(JNIEnv * env, jclass,
                                                                                    jstring userId)
{
  auto const id = jni::ToNativeString(env, userId);
  return static_cast<jboolean>(g_friends.AcceptRequest(id));
}

JNIEXPORT jboolean JNICALL Java_app_organicmaps_sdk_friends_Friends_nativeCancelRequest(JNIEnv * env, jclass,
                                                                                    jstring userId)
{
  auto const id = jni::ToNativeString(env, userId);
  return static_cast<jboolean>(g_friends.CancelRequest(id));
}
}