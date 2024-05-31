#pragma once

#include "../base/tree.h"

namespace salt {

class Refine
{
 public:
  static void cancelIntersect(Tree& tree);
  static void flip(Tree& tree);
  static void uShift(Tree& tree);  // should be after flip to achieve good quality
  static void removeRedundantCoincident(Tree& tree);
  static void substitute(Tree& tree, double eps, bool useRTree = true);

  static void Substitute_delay_aware(Tree& tree, double eps, double rd, bool useRTree = true);
  static void Substitute_delay_aware_1(Tree& tree, double eps, double rd, bool useRTree = true);
  static void Substitute_critical_delay_aware(Tree& tree, double eps, double rd, bool useRTree = true);
  static void Substitute_critical_delay_aware_1(Tree& tree, double eps, double rd, bool useRTree = true);
};

}  // namespace salt