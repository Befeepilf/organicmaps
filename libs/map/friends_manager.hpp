#pragma once

#include "base/visitor.hpp"

#include <string>
#include <vector>

struct FriendRecord
{
  std::string m_userId;
  std::string m_username;

  DECLARE_VISITOR(visitor(m_userId, "user_id"), visitor(m_username, "username"))

  bool operator==(FriendRecord const & other) const
  {
    return m_userId == other.m_userId && m_username == other.m_username;
  }
};

struct FriendsLists
{
  std::vector<FriendRecord> m_accepted;
  std::vector<FriendRecord> m_incoming;
  std::vector<FriendRecord> m_outgoing;

  DECLARE_VISITOR(visitor(m_accepted, "accepted"), visitor(m_incoming, "incoming"), visitor(m_outgoing, "outgoing"))

  bool operator==(FriendsLists const & other) const
  {
    return m_accepted == other.m_accepted && m_incoming == other.m_incoming && m_outgoing == other.m_outgoing;
  }
};

class FriendsManager
{
public:
  FriendsManager();

  bool LoadCache();
  bool SaveCache() const;
  bool EnsureCacheLoaded();

  std::string GetListsJson() const;
  FriendsLists const & GetLists() const { return m_lists; }

  void Refresh();

  std::string SearchByUsernameJson(std::string const & query);
  std::vector<FriendRecord> SearchByUsername(std::string const & query);

  bool SendRequest(std::string const & userId);
  bool AcceptRequest(std::string const & userId);
  bool CancelRequest(std::string const & userId);

private:
  std::string GetCacheFilePath() const;

  bool GetJson(std::string const & url, std::string & outJson);
  bool PostJson(std::string const & url, std::string const & body, std::string & outJson);

  FriendsLists m_lists;
  bool m_cacheLoaded = false;
};