#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "base/visitor.hpp"

// Aggregates per-region weekly exploration deltas and uploads them.
// - Aggregation continues even when sharing is disabled
// - Uploads only when sharing is enabled and server URL is configured
class ExploreStatsService
{
public:
  ExploreStatsService();

  void EnableSharing(bool enabled);
  bool IsSharingEnabled() const;

  struct StatsEntry
  {
    std::string m_regionId;
    uint64_t m_weekStartSec = 0;    // UTC week bucket start (seconds since epoch)
    uint64_t m_exploredPixels = 0;  // cumulative within the week bucket
    uint64_t m_version = 0;         // increments on local change for idempotency

    DECLARE_VISITOR_AND_DEBUG_PRINT(StatsEntry, visitor(m_regionId, "regionId"), visitor(m_weekStartSec, "weekStart"),
                                    visitor(m_exploredPixels, "explored"), visitor(m_version, "version"))
  };

  void GetEntries(std::vector<StatsEntry> & out) const;
  void ResetRegion(std::string const & regionId);

  void OnExplorationDelta(std::string const & regionId, uint32_t deltaPixels, double eventTimeSeconds);

  void TryUpload();

private:
  struct Snapshot
  {
    std::vector<StatsEntry> m_entries;
    DECLARE_VISITOR_AND_DEBUG_PRINT(Snapshot, visitor(m_entries, "entries"))
  };

  struct UploadPayload
  {
    std::string m_deviceId;
    std::string m_username;  // optional
    std::vector<StatsEntry> m_entries;
    DECLARE_VISITOR_AND_DEBUG_PRINT(UploadPayload, visitor(m_deviceId, "deviceId"), visitor(m_username, "username"),
                                    visitor(m_entries, "entries"))
  };

  std::string GetFilePath() const;

  void EnsureLoaded();
  void ScheduleSave();
  void Save();
  void SchedulePeriodicUpload();

  std::string BuildUploadJson() const;

  static uint64_t ToWeekBucket(uint64_t secondsSinceEpoch);

  static std::string MakeKey(std::string const & regionId, uint64_t weekStartSec);

private:
  mutable std::mutex m_mutex;
  std::unordered_map<std::string, StatsEntry> m_keyToEntry;
  bool m_loaded = false;
  bool m_sharingEnabled = false;
  bool m_saveScheduled = false;
  std::chrono::steady_clock::time_point m_changedAt = std::chrono::steady_clock::time_point::min();
  std::chrono::steady_clock::time_point m_lastUploadAt = std::chrono::steady_clock::time_point::min();
};