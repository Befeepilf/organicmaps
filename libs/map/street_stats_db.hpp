#pragma once

#include "indexer/mwm_set.hpp"

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

struct sqlite3;

namespace street_stats
{
// Manages a persistent SQLite database for storing street exploration statistics.
// This class is a singleton and is thread-safe.
class StreetStatsDB
{
public:
  static StreetStatsDB & Instance();

  using Bitmask = std::vector<uint8_t>;

  std::optional<Bitmask> GetBitmask(MwmSet::MwmId const & mwmId, uint32_t featureId);

  void SaveBitmask(MwmSet::MwmId const & mwmId, uint32_t featureId, Bitmask const & bitmask);

private:
  StreetStatsDB();
  ~StreetStatsDB();

  StreetStatsDB(StreetStatsDB const &) = delete;
  StreetStatsDB & operator=(StreetStatsDB const &) = delete;

  void InitSchema();

  sqlite3 * m_db = nullptr;
  std::string m_dbPath;
  std::mutex m_mutex;
};
}  // namespace street_stats