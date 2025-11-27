#include "map/friends_manager.hpp"

#include "platform/http_client.hpp"
#include "platform/platform.hpp"

#include "coding/file_reader.hpp"
#include "coding/file_writer.hpp"
#include "coding/serdes_json.hpp"
#include "coding/writer.hpp"

#include "base/logging.hpp"

#include <sstream>

namespace
{
constexpr char kFriendsCacheFile[] = "friends_cache.json";
}  // namespace

FriendsManager::FriendsManager() = default;

std::string FriendsManager::GetCacheFilePath() const { return GetPlatform().WritablePathForFile(kFriendsCacheFile); }

bool FriendsManager::LoadCache()
{
  std::string json;
  try
  {
    FileReader reader(GetCacheFilePath());
    uint64_t const size = reader.Size();
    if (size == 0)
      return false;
    json.resize(static_cast<size_t>(size));
    reader.Read(0, &json[0], json.size());
  }
  catch (...)
  {
    return false;
  }

  if (json.empty())
    return false;

  try
  {
    coding::DeserializerJson des(json);
    des(m_lists);
    m_cacheLoaded = true;
    return true;
  }
  catch (...)
  {
    LOG(LWARNING, ("Failed to parse friends cache"));
    return false;
  }
}

bool FriendsManager::EnsureCacheLoaded()
{
  if (m_cacheLoaded)
    return true;
  return LoadCache();
}

bool FriendsManager::SaveCache() const
{
  try
  {
    FileWriter writer(GetCacheFilePath());
    coding::SerializerJson<FileWriter> ser(writer);
    ser(m_lists);
    return true;
  }
  catch (...)
  {
    LOG(LWARNING, ("Failed writing", GetCacheFilePath()));
    return false;
  }
}

std::string FriendsManager::GetListsJson() const
{
  std::string jsonStr;
  using Sink = MemWriter<std::string>;
  Sink sink(jsonStr);
  coding::SerializerJson<Sink> ser(sink);
  ser(m_lists);
  return jsonStr;
}

bool FriendsManager::GetJson(std::string const & url, std::string & outJson)
{
  platform::HttpClient request(url);
  return request.RunHttpRequest(outJson);
}

bool FriendsManager::PostJson(std::string const & url, std::string const & body, std::string & outJson)
{
  platform::HttpClient request(url);
  request.SetBodyData(body, "application/json");
  return request.RunHttpRequest(outJson);
}

void FriendsManager::Refresh()
{
  // Placeholder: pull lists from server and update m_lists; then SaveCache().
}

std::string FriendsManager::SearchByUsernameJson(std::string const & query)
{
  // Placeholder: call backend /friends/search?username=query
  std::string out;
  (void)query;
  return out;
}

std::vector<FriendRecord> FriendsManager::SearchByUsername(std::string const & query)
{
  // Placeholder: returns empty until backend wired.
  (void)query;
  return {};
}

bool FriendsManager::SendRequest(std::string const & userId)
{
  (void)userId;
  return false;
}

bool FriendsManager::AcceptRequest(std::string const & userId)
{
  (void)userId;
  return false;
}

bool FriendsManager::CancelRequest(std::string const & userId)
{
  (void)userId;
  return false;
}