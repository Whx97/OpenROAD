/////////////////////////////////////////////////////////////////////////////
//
// BSD 3-Clause License
//
// Copyright (c) 2019, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#include <boost/polygon/polygon.hpp>

#include "odb/db.h"

namespace ord {
class OpenRoad;
}

namespace utl {
class Logger;
}

namespace odb {
class dbDatabase;
class dbTech;
class dbBlock;
}  // namespace odb

namespace tap {

struct Options
{
  odb::dbMaster* endcap_master = nullptr;
  odb::dbMaster* tapcell_master = nullptr;
  int dist = -1;    // default = 2um
  int halo_x = -1;  // default = 2um
  int halo_y = -1;  // default = 2um
  odb::dbMaster* cnrcap_nwin_master = nullptr;
  odb::dbMaster* cnrcap_nwout_master = nullptr;
  odb::dbMaster* tap_nwintie_master = nullptr;
  odb::dbMaster* tap_nwin2_master = nullptr;
  odb::dbMaster* tap_nwin3_master = nullptr;
  odb::dbMaster* tap_nwouttie_master = nullptr;
  odb::dbMaster* tap_nwout2_master = nullptr;
  odb::dbMaster* tap_nwout3_master = nullptr;
  odb::dbMaster* incnrcap_nwin_master = nullptr;
  odb::dbMaster* incnrcap_nwout_master = nullptr;

  bool addBoundaryCells() const
  {
    return tap_nwintie_master && tap_nwin2_master && tap_nwin3_master
           && tap_nwouttie_master && tap_nwout2_master && tap_nwout3_master
           && incnrcap_nwin_master && incnrcap_nwout_master;
  }
};

struct BoundaryCellOptions
{
  // External facing boundary cells
  odb::dbMaster* outer_corner_top_left_r0 = nullptr;
  odb::dbMaster* outer_corner_top_left_mx = nullptr;

  odb::dbMaster* outer_corner_top_right_r0 = nullptr;
  odb::dbMaster* outer_corner_top_right_mx = nullptr;

  odb::dbMaster* outer_corner_bottom_left_r0 = nullptr;
  odb::dbMaster* outer_corner_bottom_left_mx = nullptr;

  odb::dbMaster* outer_corner_bottom_right_r0 = nullptr;
  odb::dbMaster* outer_corner_bottom_right_mx = nullptr;

  // Internal facing boundary cells
  odb::dbMaster* inner_corner_top_left_r0 = nullptr;
  odb::dbMaster* inner_corner_top_left_mx = nullptr;

  odb::dbMaster* inner_corner_top_right_r0 = nullptr;
  odb::dbMaster* inner_corner_top_right_mx = nullptr;

  odb::dbMaster* inner_corner_bottom_left_r0 = nullptr;
  odb::dbMaster* inner_corner_bottom_left_mx = nullptr;

  odb::dbMaster* inner_corner_bottom_right_r0 = nullptr;
  odb::dbMaster* inner_corner_bottom_right_mx = nullptr;

  // endcaps
  std::vector<odb::dbMaster*> top_r0;
  std::vector<odb::dbMaster*> top_mx;

  std::vector<odb::dbMaster*> bottom_r0;
  std::vector<odb::dbMaster*> bottom_mx;

  odb::dbMaster* left_r0 = nullptr;
  odb::dbMaster* left_mx = nullptr;

  odb::dbMaster* right_r0 = nullptr;
  odb::dbMaster* right_mx = nullptr;

  std::string prefix = "PHY_";
};

class Tapcell
{
 public:
  Tapcell();
  void init(odb::dbDatabase* db, utl::Logger* logger);
  void setTapPrefix(const std::string& tap_prefix);
  void setEndcapPrefix(const std::string& endcap_prefix);
  void clear();
  void run(const Options& options);
  void cutRows(const Options& options);
  void reset();
  int removeCells(const std::string& prefix);

  void insertBoundaryCells(const BoundaryCellOptions& options);
  void insertTapcells(const Options& options);

 private:
  struct FilledSites
  {
    int yMin = 0;
    int xMin = 0;
    int xMax = 0;
  };
  using RowFills = std::map<int, std::vector<std::vector<int>>>;

  std::vector<odb::dbBox*> findBlockages();
  const std::pair<int, int> getMinMaxX(
      const std::vector<std::vector<odb::dbRow*>>& rows);
  RowFills findRowFills();
  bool checkSymmetry(odb::dbMaster* master, const odb::dbOrientType& ori);
  odb::dbInst* makeInstance(odb::dbBlock* block,
                            odb::dbMaster* master,
                            const odb::dbOrientType& orientation,
                            int x,
                            int y,
                            const std::string& prefix);
  bool checkIfFilled(int x,
                     int width,
                     const odb::dbOrientType& orient,
                     const std::vector<std::vector<int>>& row_insts);
  std::map<std::pair<int, int>, std::vector<int>> getMacroOutlines(
      const std::vector<std::vector<odb::dbRow*>>& rows);
  std::vector<std::vector<odb::dbRow*>> organizeRows();
  int insertTapcells(const std::vector<std::vector<odb::dbRow*>>& rows,
                     odb::dbMaster* tapcell_master,
                     int dist);

  int defaultDistance() const;

  using Polygon = boost::polygon::polygon_90_data<int>;
  using Polygon90 = boost::polygon::polygon_90_with_holes_data<int>;
  std::vector<Polygon90> getBoundaryAreas() const;
  enum class EdgeType
  {
    Left,
    Top,
    Right,
    Bottom,
    Unknown
  };
  struct Edge
  {
    EdgeType type;
    odb::Point pt0;
    odb::Point pt1;
  };
  std::vector<Edge> getBoundaryEdges(const Polygon90& area, bool outer) const;
  enum class CornerType
  {
    OuterBottomLeft,
    OuterTopLeft,
    OuterTopRight,
    OuterBottomRight,
    InnerBottomLeft,
    InnerTopLeft,
    InnerTopRight,
    InnerBottomRight,
    Unknown
  };
  struct Corner
  {
    CornerType type;
    odb::Point pt;
  };
  std::vector<Corner> getBoundaryCorners(const Polygon90& area,
                                         bool outer) const;

  std::string toString(EdgeType type) const;
  std::string toString(CornerType type) const;

  odb::dbRow* getRow(const Corner& corner, odb::dbSite* site) const;
  std::vector<odb::dbRow*> getRows(const Edge& edge, odb::dbSite* site) const;

  std::pair<int, int> insertBoundaryCells(const Polygon& area,
                                          bool outer,
                                          const BoundaryCellOptions& options);
  std::pair<int, int> insertBoundaryCells(const Polygon90& area,
                                          bool outer,
                                          const BoundaryCellOptions& options);

  using CornerMap = std::map<odb::dbRow*, std::set<odb::dbInst*>>;
  CornerMap insertBoundaryCorner(const Corner& corner,
                                 const BoundaryCellOptions& options);
  int insertBoundaryEdge(const Edge& edge,
                         const CornerMap& corners,
                         const BoundaryCellOptions& options);
  int insertBoundaryEdgeHorizontal(const Edge& edge,
                                   const CornerMap& corners,
                                   const BoundaryCellOptions& options);
  int insertBoundaryEdgeVertical(const Edge& edge,
                                 const CornerMap& corners,
                                 const BoundaryCellOptions& options);

  BoundaryCellOptions correctBoundaryOptions(
      const BoundaryCellOptions& options) const;

  BoundaryCellOptions correctBoundaryOptions(const Options& options) const;

  void initFilledSites();

  odb::dbDatabase* db_ = nullptr;
  utl::Logger* logger_ = nullptr;
  int phy_idx_ = 0;
  std::vector<FilledSites> filled_sites_;
  std::string tap_prefix_;
  std::string endcap_prefix_;
};

}  // namespace tap
