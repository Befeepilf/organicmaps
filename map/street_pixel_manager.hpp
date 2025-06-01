#pragma once

#include "map/bookmark_manager.hpp"
#include "map/user_mark.hpp"
#include "map/user_mark_layer.hpp"

#include "drape_frontend/drape_engine_safe_ptr.hpp"
#include "drape_frontend/user_marks_provider.hpp"

#include "drape/color.hpp"

#include "geometry/point2d.hpp"

#include "indexer/features_vector.hpp"

#include "storage/storage.hpp"

#include <healpix_base.h>

#include <map>
#include <memory>
#include <set>
#include <vector>

// A very small manager that owns UserMark::STREET_PIXEL marks and exposes them
// to drape for rendering.  It is intentionally kept minimal – no categories,
// no persistence, just an in-memory container that can be populated with point
// coordinates and then rendered.
class StreetPixelManager : public df::UserMarksProvider
{
public:
  // A tiny helper mark that represents a single street-pixel.  It is rendered
  // as a fixed-size coloured circle.
  class PixelMark final : public ColoredMarkPoint
  {
  public:
    explicit PixelMark(::m2::PointD const & pt)
      : ColoredMarkPoint(pt)
    {
      // Green dot, 4 px radius.
      SetColor(dp::Color(0, 255, 0, 255));
      SetRadius(4.0f);
    }

    ~PixelMark() override = default;

    bool IsAvailableForSearch() const override { return false; }
  };

  StreetPixelManager();

  void SetDrapeEngine(ref_ptr<df::DrapeEngine> engine);

  void SetBookmarkManager(BookmarkManager * bmManager);

  void LoadStreetPixelsForRegion(storage::CountryId const & countryId, storage::LocalFilePtr const & localFile);

  void DeriveStreetPixelsFromFeatures(FeaturesVectorTest & featuresVector, std::vector<int64> & streetPixels);

  // points are expected in Mercator coordinates
  void AddPixels(std::vector<::m2::PointD> const & points);

  kml::GroupIdSet const & GetUpdatedGroupIds() const override { return m_updatedGroups; }
  kml::GroupIdSet const & GetRemovedGroupIds() const override { return m_removedGroups; }
  kml::GroupIdSet GetAllGroupIds() const override { return m_allGroups; }
  kml::GroupIdSet const & GetBecameVisibleGroupIds() const override { return m_visibleGroups; }
  kml::GroupIdSet const & GetBecameInvisibleGroupIds() const override { return m_invisibleGroups; }
  bool IsGroupVisible(kml::MarkGroupId /*groupId*/) const override { return true; }
  kml::MarkIdSet const & GetGroupPointIds(kml::MarkGroupId /*groupId*/) const override { return m_allMarks; }
  kml::TrackIdSet const & GetGroupLineIds(kml::MarkGroupId /*groupId*/) const override { return m_dummyLines; }
  kml::MarkIdSet const & GetCreatedMarkIds() const override { return m_createdMarks; }
  kml::MarkIdSet const & GetRemovedMarkIds() const override { return m_removedMarks; }
  kml::MarkIdSet const & GetUpdatedMarkIds() const override { return m_updatedMarks; }
  kml::TrackIdSet const & GetCreatedLineIds() const override { return m_dummyLines; }
  kml::TrackIdSet const & GetRemovedLineIds() const override { return m_dummyLines; }
  kml::TrackIdSet const & GetUpdatedLineIds() const override { return m_dummyLines; }
  df::UserPointMark const * GetUserPointMark(kml::MarkId id) const override;
  df::UserLineMark const * GetUserLineMark(kml::TrackId /*lineId*/) const override { return nullptr; }

private:
  void NotifyDrape();

  df::DrapeEngineSafePtr m_drapeEngine;
  bool m_firstNotify = true;

  BookmarkManager * m_bmManager = nullptr;

  T_Healpix_Base<int64> m_healpixBase;

  std::unique_ptr<UserMarkLayer> m_layer;
  std::map<kml::MarkId, std::unique_ptr<PixelMark>> m_marks;

  kml::GroupIdSet m_allGroups{static_cast<kml::MarkGroupId>(UserMark::STREET_PIXEL)};
  kml::GroupIdSet m_updatedGroups;
  kml::GroupIdSet m_removedGroups;
  kml::GroupIdSet m_visibleGroups;
  kml::GroupIdSet m_invisibleGroups;

  kml::MarkIdSet m_allMarks;
  kml::MarkIdSet m_createdMarks;
  kml::MarkIdSet m_removedMarks;
  kml::MarkIdSet m_updatedMarks;

  kml::TrackIdSet m_dummyLines;  // always empty
};
