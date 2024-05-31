#pragma once

#include "../base/tree.h"
#include "../base/eval.h"

// #include "api/TimingEngine.hh"
// #include "RoutingTree.h"
// using PinLoc = std::tuple<ista::DesignObject*, ito::Point>;
namespace salt {

class DelayRefine
{
 public:
  // static void cancelIntersect(Tree& tree);
  // static void flip(Tree& tree);
  // static void uShift(Tree& tree);  // should be after flip to achieve good quality
  // static void substitute(Tree& tree, double eps, bool useRTree = true);

  static shared_ptr<TreeNode> findLowestCommonAncestor(shared_ptr<TreeNode> p, shared_ptr<TreeNode> q);
  static double calc_delay_delta(shared_ptr<TreeNode> p,
                        shared_ptr<TreeNode> q,
                        shared_ptr<TreeNode> critical,
                        shared_ptr<TreeNode> node,
                        shared_ptr<TreeNode> neighbor,
                        Point steinerPt_loc,
                        vector<double> node_cap,
                        vector<DTYPE> pathLengths,
                        DTYPE wireLengthDelta);

  static void Substitute_delay_aware(Tree& tree, double eps, double rd, bool useRTree = true);
  // static void Substitute_delay_aware_1(Tree& tree, double eps, double rd, ista::TimingEngine *_timing_engine, ista::Net *sta_net, const vector<PinLoc> pin2loc, bool useRTree = true);
  static void Substitute_critical_delay_aware(Tree& tree, double eps, double rd, bool useRTree = true);
  static void Substitute_critical_delay_aware_1(Tree& tree, double eps, double rd, bool useRTree = true);
};

}  // namespace salt