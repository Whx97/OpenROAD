#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "delay_refine.h"
#include "../base/eval.h"
#include "../base/mst.h"

// #include "api/TimingEngine.hh"
// #include "api/TimingIDBAdapter.hh"

// #include "Net.h"

// #include "RoutingTree.h"
namespace salt {

std::set<int> visited_node;

bool salt_delay = 1;

// void RctNodeConnectPin(ista::Net *net, int index, ista::RctNode *rcnode, int num_pins,
//                        ista::TimingEngine       *_timing_engine,
//                        const vector<PinLoc> pin2loc);
// double time(ista::TimingEngine *_timing_engine, ista::Net *curr_net, salt::Tree& tree, const vector<PinLoc> pin2loc) {
//     if (_timing_engine->get_ista()->getRcNet(curr_net)) {
//         _timing_engine->resetRcTree(curr_net);
//     }

//     // for (auto p : pin2loc) {
//     //     auto a = get<0>(p);
//     //     cout << a->getFullName() << endl;
//     // }
    
//     std::vector<std::pair<int, int>> wire_segment_idx;
//     std::vector<int>                 length_per_wire;
//     auto update_wire = [&](const shared_ptr<TreeNode>& node) {
//       if (node->parent) {
//         length_per_wire.push_back(node->WireToParent());
//         wire_segment_idx.push_back({node->id, node->parent->id});
//       }
//     };
//     tree.postOrder(update_wire);

//     int num_pins = tree.net->pins.size();

//     int numb = wire_segment_idx.size();
//     for (int i = 0; i != numb; ++i) {
//       int      index1 = wire_segment_idx[i].first;
//       int      index2 = wire_segment_idx[i].second;
//       ista::RctNode *n1 = _timing_engine->makeOrFindRCTreeNode(curr_net, index1);
//       ista::RctNode *n2 = _timing_engine->makeOrFindRCTreeNode(curr_net, index2);

//       int length_dbu = length_per_wire[i];
//       if (length_dbu == 0) {
//         _timing_engine->makeResistor(curr_net, n1, n2, 1.0e-3);
//       } else {
//         std::optional<double> width = std::nullopt;
//         double                cap = dynamic_cast<ista::TimingIDBAdapter *>(_timing_engine->get_db_adapter())
//                          ->getCapacitance(1, (double)length_dbu / 2000, width);
//         double res = dynamic_cast<ista::TimingIDBAdapter *>(_timing_engine->get_db_adapter())
//                          ->getResistance(1, (double)length_dbu / 2000, width);
//         // double unitCap = dynamic_cast<ista::TimingIDBAdapter *>(_timing_engine->get_db_adapter())
//         //             ->getCapacitance(1, 1.0/1000, width);

//         // // unitCapacitance = cap;
//         // double unitRes = dynamic_cast<ista::TimingIDBAdapter *>(_timing_engine->get_db_adapter())
//         //                   ->getResistance(1, 1.0/1000, width);
//         // double cap = unitCap * length_dbu;
//         // double res = unitRes * length_dbu;

//         if (curr_net->isClockNet()) {
//           cap /= 10.0;
//           res /= 10.0;
//         // } else {
//         //   cap /= 2.0;
//         //   res /= 2.0;
//         }

//         _timing_engine->incrCap(n1, cap / 2.0, true);
//         _timing_engine->makeResistor(curr_net, n1, n2, res);
//         _timing_engine->incrCap(n2, cap / 2.0, true);
//       }
//       RctNodeConnectPin(curr_net, index1, n1, num_pins, _timing_engine, pin2loc);
//       RctNodeConnectPin(curr_net, index2, n2, num_pins, _timing_engine, pin2loc);
//     }

//     // cout << "net " << curr_net->get_name() << endl;

//     _timing_engine->updateRCTreeInfo(curr_net);
//     auto rc_net = _timing_engine->get_ista()->getRcNet(curr_net);
//     auto* rc_tree = rc_net->rct();

//     double sum_delay = 0.0;
//     auto loads = curr_net->getLoads();
//     for (auto load : loads) {
//         auto pin_name = load->getFullName();
//         auto de = rc_tree->delay(pin_name);
//         // sum_delay += de;
//         sum_delay = max(sum_delay, de);
//         // cout << "delay: " << de;
//         // cout << endl;
//     }

//     if (_timing_engine->get_ista()->getRcNet(curr_net)) {
//         _timing_engine->resetRcTree(curr_net);
//     }
//     visited_node.clear();
//     return sum_delay;
// }

// void RctNodeConnectPin(ista::Net *net, int index, ista::RctNode *rcnode, int num_pins,
//                        ista::TimingEngine       *_timing_engine,
//                        const vector<PinLoc> pin2loc) {
//   //   int num_pins = tree->get_pins().size();
//   //   if (tree->get_pin_visit(index) == 1) {
//   //     return;
//   //   }
//   if (visited_node.find(index) != visited_node.end()) {
//     return;
//   }
//   if (index < num_pins) {
//     // tree->set_pin_visit(index);
//     visited_node.insert(index);
//     ista::RctNode *pin_node = _timing_engine->makeOrFindRCTreeNode(get<0>(pin2loc[index]));
//     if (index == 0) {
//       _timing_engine->makeResistor(net, pin_node, rcnode, 1.0e-3);
//     } else {
//       _timing_engine->makeResistor(net, rcnode, pin_node, 1.0e-3);
//     }
//   }
// }

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using BPoint = bg::model::point<DTYPE, 2, bg::cs::cartesian>;
using BSegment = bg::model::segment<BPoint>;
using BBox = bg::model::box<BPoint>;
using BPolygon = bg::model::polygon<BPoint>;
using RNode = pair<BBox, shared_ptr<TreeNode>>;  // R-Tree node
struct RNodeComp
{
  bool operator()(const RNode& l, const RNode& r) const { return bg::equals(l.first, r.first) && l.second == r.second; }
};

// void DelayRefine::substitute(Tree& tree, double eps, bool useRTree)
// {
//   bgi::rtree<RNode, bgi::rstar<8>, bgi::indexable<RNode>, RNodeComp> rtree;
//   if (useRTree) {
//     tree.postOrder([&](const shared_ptr<TreeNode>& n) {
//       if (n->parent) {
//         BBox s;
//         bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(n->parent->loc.x, n->parent->loc.y)), s);
//         rtree.insert({s, n});
//       }
//     });
//   }
//   auto disconnect = [&](const shared_ptr<TreeNode>& n) {
//     if (useRTree) {
//       BBox s;
//       bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(n->parent->loc.x, n->parent->loc.y)), s);
//       rtree.remove({s, n});
//     }
//     TreeNode::resetParent(n);
//   };
//   auto connect = [&](const shared_ptr<TreeNode>& n, const shared_ptr<TreeNode>& parent) {
//     TreeNode::setParent(n, parent);
//     if (useRTree) {
//       BBox s;
//       bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(parent->loc.x, parent->loc.y)), s);
//       rtree.insert({s, n});
//     }
//   };
//   while (true) {
//     // Get nearest neighbors
//     tree.UpdateId();
//     vector<shared_ptr<TreeNode>> nodes = tree.ObtainNodes(),
//                                  ordered_nodes(nodes.size());  // note: all pins should be covered
//     vector<Point> points(nodes.size());
//     for (int i = 0; i < nodes.size(); ++i) {
//       ordered_nodes[nodes[i]->id] = nodes[i];
//       points[nodes[i]->id] = nodes[i]->loc;  // TODO: move within bracket
//     }
//     nodes = ordered_nodes;
//     vector<vector<int>> nearest_neighbors;
//     if (!useRTree) {
//       MstBuilder mst_builder;
//       mst_builder.GetAllNearestNeighbors(points, nearest_neighbors);
//     } else {
//       nearest_neighbors.resize(nodes.size());
//       for (auto n : nodes) {
//         if (n->parent) {
//           Point c = n->loc;  // center
//           DTYPE radius = n->WireToParent();
//           // diamond is too slow...
//           // BPolygon diamond;
//           // diamond.outer().emplace_back(c.x - radius, c.y);
//           // diamond.outer().emplace_back(c.x, c.y + radius);
//           // diamond.outer().emplace_back(c.x + radius, c.y);
//           // diamond.outer().emplace_back(c.x, c.y - radius);
//           // diamond.outer().emplace_back(c.x - radius, c.y);
//           BBox query_box{{c.x - radius, c.y - radius}, {c.x + radius, c.y + radius}};
//           vector<RNode> cands;
//           rtree.query(bgi::intersects(query_box), back_inserter(cands));  // TODO: change back_inserter
//           for (const auto& cand : cands) {
//             nearest_neighbors[n->id].push_back(cand.second->id);
//           }
//         }
//       }
//     }

//     // Prune descendants in nearest neighbors
//     vector<int> pre_order_idxes(nodes.size(), -1);
//     int global_pre_order_idx = 0;
//     function<void(const shared_ptr<TreeNode>&)> remove_descendants = [&](const shared_ptr<TreeNode>& node) {
//       pre_order_idxes[node->id] = global_pre_order_idx++;
//       for (auto child : node->children) {
//         remove_descendants(child);
//       }
//       for (auto& neigh_idx : nearest_neighbors[node->id]) {
//         int neigh_pre_order_idx = pre_order_idxes[neigh_idx];
//         if (neigh_pre_order_idx != -1 && neigh_pre_order_idx >= pre_order_idxes[node->id]) {
//           neigh_idx = -1;  // -1 stands for "descendant"
//         }
//       }
//     };
//     remove_descendants(tree.source);

//     // Init path lengths and subtree slacks
//     vector<DTYPE> path_lengths(nodes.size());
//     vector<DTYPE> slacks(nodes.size());
//     auto update_path_lengths = [&](const shared_ptr<TreeNode>& node) {
//       if (node->parent) {
//         path_lengths[node->id] = path_lengths[node->parent->id] + node->WireToParent();
//       } else {
//         path_lengths[node->id] = 0;
//       }
//     };
//     auto update_slacks = [&](const shared_ptr<TreeNode>& node) {
//       if (node->children.empty()) {
//         slacks[node->id] = Dist(node->loc, tree.source->loc) * (1 + eps) - path_lengths[node->id];  // floor here...
//       } else {
//         DTYPE min_slack = Dist(node->loc, tree.source->loc) * (1 + eps) - path_lengths[node->id];
//         for (auto child : node->children) {
//           min_slack = min(min_slack, slacks[child->id]);
//         }
//         slacks[node->id] = min_slack;
//       }
//     };
//     tree.preOrder(update_path_lengths);
//     tree.postOrder(update_slacks);

//     // Find legal candidate moves
//     using MoveT = tuple<DTYPE, shared_ptr<TreeNode>, shared_ptr<TreeNode>>;
//     vector<MoveT> candidate_moves;  // <wire_length_delta, node, newParent>
//     auto get_nearest_point = [](const shared_ptr<TreeNode>& target, const shared_ptr<TreeNode>& neigh) {
//       Box box(neigh->loc, neigh->parent->loc);
//       box.Legalize();
//       return box.GetNearestPointTo(target->loc);
//     };
//     for (auto node : nodes) {
//       if (!(node->parent)) {
//         continue;
//       }
//       DTYPE best_wire_length_delta = 0;  // the negative, the better
//       shared_ptr<TreeNode> best_new_parent;
//       for (int neigh_idx : nearest_neighbors[node->id]) {
//         if (neigh_idx == -1 || !nodes[neigh_idx]->parent)
//           continue;
//         auto neigh = nodes[neigh_idx];
//         auto neigh_parent = neigh->parent;
//         auto steiner_pt = get_nearest_point(node, neigh);
//         DTYPE wire_length_delta = Dist(node->loc, steiner_pt) - node->WireToParent();
//         if (wire_length_delta < best_wire_length_delta) {  // has wire length improvement
//           DTYPE path_length_delta = path_lengths[neigh_parent->id] + Dist(node->loc, neigh_parent->loc) - path_lengths[node->id];
//           if (path_length_delta <= slacks[node->id]) {  // make path length under control
//             best_wire_length_delta = wire_length_delta;
//             best_new_parent = neigh;
//           }
//         }
//       }
//       if (best_new_parent) {
//         candidate_moves.emplace_back(best_wire_length_delta, node, best_new_parent);
//       }
//     }
//     if (candidate_moves.empty()) {
//       break;
//     }

//     // Try candidate moves in the order of descending wire length savings
//     // Note that earlier moves may influence the legality of later one
//     sort(candidate_moves.begin(), candidate_moves.end(), [](const MoveT& lhs, const MoveT& rhs) { return get<0>(lhs) < get<0>(rhs); });
//     for (const auto& move : candidate_moves) {
//       auto node = get<1>(move), neigh = get<2>(move);
//       auto neigh_parent = neigh->parent;
//       // check due to earlier moves
//       if (TreeNode::isAncestor(node, neigh_parent))
//         continue;
//       DTYPE path_length_delta = path_lengths[neigh_parent->id] + Dist(node->loc, neigh_parent->loc) - path_lengths[node->id];
//       if (path_length_delta > slacks[node->id])
//         continue;
//       auto steiner_pt = get_nearest_point(node, neigh);
//       DTYPE wire_length_delta = Dist(node->loc, steiner_pt) - node->WireToParent();
//       if (wire_length_delta >= 0)
//         continue;
//       // break
//       disconnect(node);
//       // reroot
//       if (steiner_pt == neigh->loc) {
//         connect(node, neigh);
//       } else if (steiner_pt == neigh_parent->loc) {
//         connect(node, neigh_parent);
//       } else {
//         auto steiner_node = make_shared<TreeNode>(steiner_pt);
//         connect(steiner_node, neigh_parent);
//         disconnect(neigh);
//         connect(neigh, steiner_node);
//         connect(node, steiner_node);
//         // for later moves
//         steiner_node->id = nodes.size();
//         nodes.push_back(steiner_node);
//         path_lengths.push_back(path_lengths[neigh_parent->id] + steiner_node->WireToParent());
//         slacks.push_back(Dist(steiner_node->loc, tree.source->loc) * (1 + eps) - path_lengths.back());
//       }
//       // update slack for later moves: first subtree, then path to source
//       TreeNode::preOrder(neigh_parent, update_path_lengths);
//       TreeNode::postOrder(neigh_parent, update_slacks);
//       auto tmp = neigh_parent;
//       while (tmp->parent) {
//         slacks[tmp->parent->id] = min(slacks[tmp->parent->id], slacks[tmp->id]);
//         tmp = tmp->parent;
//       }
//     }

//     // Finalize
//     // tree.RemoveTopoRedundantSteiner();
//     tree.postOrderCopy([&](const shared_ptr<TreeNode>& node) {
//       // degree may change after post-order traversal of its children
//       if (node->pin)
//         return;
//       if (node->children.empty()) {
//         disconnect(node);
//       } else if (node->children.size() == 1) {
//         auto old_parent = node->parent, old_child = node->children[0];
//         disconnect(node);
//         disconnect(old_child);
//         connect(old_child, old_parent);
//       }
//     });
//   }
// }

// void DelayRefine::Substitute_delay_aware_1(Tree &tree, double eps, double rd,
//                                            ista::TimingEngine *_timing_engine,
//                                            ista::Net *sta_net, 
//                                            const vector<PinLoc> pin2loc,bool useRTree) {
//   bgi::rtree<RNode, bgi::rstar<8>, bgi::indexable<RNode>, RNodeComp> rtree;
//   if (useRTree) {
//     tree.postOrder([&](const shared_ptr<TreeNode> &n) {
//       if (n->parent) {
//         BBox s;
//         bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y),
//                               BPoint(n->parent->loc.x, n->parent->loc.y)),
//                      s);
//         rtree.insert({s, n});
//       }
//     });
//   }
//   auto Disconnect = [&](const shared_ptr<TreeNode> &n) {
//     if (useRTree) {
//       BBox s;
//       bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y),
//                             BPoint(n->parent->loc.x, n->parent->loc.y)),
//                    s);
//       rtree.remove({s, n});
//     }
//     TreeNode::resetParent(n);
//   };
//   auto Connect = [&](const shared_ptr<TreeNode> &n, const shared_ptr<TreeNode> &parent) {
//     TreeNode::setParent(n, parent);
//     if (useRTree) {
//       BBox s;
//       bg::envelope(
//           BSegment(BPoint(n->loc.x, n->loc.y), BPoint(parent->loc.x, parent->loc.y)), s);
//       rtree.insert({s, n});
//     }
//   };

// //   auto make_node_id = 10 * tree.UpdateId();

// //   auto outchild = [&](const shared_ptr<TreeNode> &node) {
// //     if (node->children.empty()) {
// //       cout << "Node " << node->id << " has no children" << endl;
// //     } else {
// //       cout << "Node " << node->id << " has children: ";
// //       for (auto c : node->children) {
// //         cout << c->id << " ";
// //       }
// //       cout << endl;
// //     }
// //   };

//   // 预备
//   double for_one_net_maxLB;
//   double before_sumDelay;
//   if (salt_delay) {
//     salt::ElmoreDelayEval before_delayEval(rd, tree);
//     before_sumDelay = before_delayEval.sumNorDelay;
//     for_one_net_maxLB = before_delayEval._maxLb;
//   } else {
//     before_sumDelay = time(_timing_engine, sta_net, tree, pin2loc);
//   }



//   int iter_num = 0;

//   // comment_info 统计改变topology和计算delay所用的时间
//   TimeStat time_stat;

//   while (true) {
//     iter_num++;
//         // Get nearest neighbors
//         int num = tree.UpdateId();
//         cout << "num: " << num << endl;
//         vector<shared_ptr<TreeNode>> nodes = tree.ObtainNodes(),
//                                      orderedNodes(nodes.size());  // note: all pins should be covered
//         if (nodes.size() != num) {
//             cout << "debug\n";
//         }
//         vector<Point> points(nodes.size());
//         for (int i = 0; i < nodes.size(); ++i) {
//             // cout << "node " << nodes[i]->id << endl;
//             orderedNodes[nodes[i]->id] = nodes[i];
//             points[nodes[i]->id] = nodes[i]->loc;  // TODO: move within bracket
//         }
//         nodes = orderedNodes;
//         vector<vector<int>> nearestNeighbors;
//         if (!useRTree) {
//             MstBuilder mstB;
//             mstB.GetAllNearestNeighbors(points, nearestNeighbors);
//         } else {
//             nearestNeighbors.resize(nodes.size());
//             for (auto n : nodes) {
//                 if (n->parent) {
//                     Point c = n->loc;  // center
//                     DTYPE radius = n->WireToParent();
//                     int tora = 1;
//                     BBox queryBox{{c.x - tora*radius, c.y - tora*radius}, {c.x + tora*radius, c.y + tora*radius}};
//                     vector<RNode> cands;
//                     rtree.query(bgi::intersects(queryBox), back_inserter(cands));  // TODO: change back_inserter
//                     for (const auto& cand : cands) {
//                         nearestNeighbors[n->id].push_back(cand.second->id);
//                     }
//                 }
//             }
//         }

//         // Prune descendants in nearest neighbors
//         vector<int> preOrderIdxes(nodes.size(), -1);
//         int globalPreOrderIdx = 0;
//         function<void(const shared_ptr<TreeNode>&)> removeDescendants = [&](const shared_ptr<TreeNode>& node) {
//             preOrderIdxes[node->id] = globalPreOrderIdx++;
//             for (auto child : node->children) {
//                 removeDescendants(child);
//             }
//             for (auto& neighIdx : nearestNeighbors[node->id]) {
//                 int neighPreOrderIdx = preOrderIdxes[neighIdx];
//                 if (neighPreOrderIdx != -1 && neighPreOrderIdx >= preOrderIdxes[node->id]) {
//                     neighIdx = -1;  // -1 stands for "descendant"
//                 }
//             }
//         };
//         removeDescendants(tree.source);

//         // Init path lengths and subtree slacks
//         vector<DTYPE> pathLengths(nodes.size());
//         vector<DTYPE> slacks(nodes.size());
//         auto UpdatePathLengths = [&](const shared_ptr<TreeNode>& node) {
//             if (node->parent) {
//                 pathLengths[node->id] = pathLengths[node->parent->id] + node->WireToParent();
//             } else {
//                 pathLengths[node->id] = 0;
//             }
//         };
//         auto UpdateSlacks = [&](const shared_ptr<TreeNode>& node) {
//             if (node->children.empty()) {
//                 slacks[node->id] =
//                     Dist(node->loc, tree.source->loc) * (1 + eps) - pathLengths[node->id];  // floor here...
//             } else {
//                 DTYPE minSlack = Dist(node->loc, tree.source->loc) * (1 + eps) - pathLengths[node->id];
//                 for (auto child : node->children) {
//                     minSlack = min(minSlack, slacks[child->id]);
//                 }
//                 slacks[node->id] = minSlack;
//             }
//         };
//         tree.preOrder(UpdatePathLengths);
//         tree.postOrder(UpdateSlacks);

//         // Find legal candidate moves
//         // using MoveT = tuple<DTYPE, shared_ptr<TreeNode>, shared_ptr<TreeNode>>;
//         using MoveT = tuple<double, shared_ptr<TreeNode>, shared_ptr<TreeNode>, int>;  // 最后一个的int，1表示选neigh，2表示选neighParent，3表示选steinerPt
//         int which_node = 0;
//         vector<MoveT> candidateMoves;  // <wireLengthDelta, node, newParent>
//         auto GetNearestPoint = [](const shared_ptr<TreeNode>& target, const shared_ptr<TreeNode>& neigh) {
//             Box box(neigh->loc, neigh->parent->loc);
//             box.Legalize();
//             return box.GetNearestPointTo(target->loc);
//         };

//         // vector<double> cap(num, 0);
//         // tree.postOrder([&](const shared_ptr<TreeNode>& node) {
//         //     if (node->pin && node != tree.source) cap[node->id] = node->pin->cap;
//         //     for (auto c : node->children) {
//         //         cap[node->id] += cap[c->id];
//         //         cap[node->id] += c->WireToParent() * 8e-20;
//         //     }
//         // });

//         for (auto node : nodes) {
//             std::shared_ptr<salt::TreeNode> tmp_steinerNode = nullptr;
//             if (!(node->parent)) {
//                 continue;
//             }

//             auto originalParent = node->parent;
//             DTYPE bestWireLengthDelta = 0;  // the negative, the better

//             double best_delay_delta = 0;

//             shared_ptr<TreeNode> bestNewParent;
//             for (int neighIdx : nearestNeighbors[node->id]) {
//                 if (node->id == 2 && neighIdx == 41) {
//                     cout << "afadsfn";
//                 }
//                 // bug   why this?
//                 if (neighIdx >= nodes.size()) continue;
//                 // bug   why this?
//                 if (neighIdx == -1 || !nodes[neighIdx]->parent) continue;
//                 auto neigh = nodes[neighIdx];
//                 auto neighParent = neigh->parent;

//                 // bug   why this?
//                 if (node == neighParent) continue;
//                 if (node == neigh) continue;
//                 // bug   why this?

//                 // pl不能违规
//                 DTYPE pathLengthDelta =
//                         pathLengths[neighParent->id] + Dist(node->loc, neighParent->loc) - pathLengths[node->id];
//                 if (pathLengthDelta > slacks[node->id]) continue;

//                 auto steinerPt = GetNearestPoint(node, neigh);

//                 bool exct_twice = (steinerPt == neigh->loc) || (steinerPt == neighParent->loc) ? 1 : 0;
//                 vector<shared_ptr<TreeNode>> tmp_nodes;
//                 tmp_nodes.emplace_back(neigh);
//                 tmp_nodes.emplace_back(neighParent);
//                 for (int i = 0; i < 2; i++) {
//                     // comment_info 改变topology所花费的时间
//                     utils::timer topo_change_Timer;
//                     TreeNode::resetParent(node);
//                     TreeNode::setParent(node, tmp_nodes[i]);
//                     double topo_change_time = topo_change_Timer.elapsed();
//                     time_stat.Inc_change_topo(topo_change_time);
//                     // comment_info 改变topology所花费的时间

//                     if (i == 0) {
//                         // pl不能违规
//                         DTYPE pathLengthDelta =
//                                 pathLengths[tmp_nodes[i]->id] + Dist(node->loc, tmp_nodes[i]->loc) - pathLengths[node->id];
//                         if (pathLengthDelta > slacks[node->id]) continue;
//                     }

//                     // comment_info 计算delay所花费的时间
//                     utils::timer delay_Timer;

//                     double sumDelay = 0;
//                     if (salt_delay) {
//                         // salt::ElmoreDelayEval delayEval(rd, tree);
//                         salt::ElmoreDelayEval delayEval(rd, tree, for_one_net_maxLB);
//                         // auto sumDelay = 100;//delayEval.sumNorDelay;
//                         sumDelay = delayEval.sumNorDelay;
//                     } else {
//                         sumDelay = time(_timing_engine, sta_net, tree, pin2loc);
//                     }
                    
//                     double delay_time = delay_Timer.elapsed();
//                     time_stat.Inc_calc_delay(delay_time);
//                     // comment_info 计算delay所花费的时间

//                     if (sumDelay + 1e-12 < before_sumDelay) {
//                         auto delay_delta = sumDelay - before_sumDelay;
//                         if (delay_delta < best_delay_delta) {
//                             best_delay_delta = delay_delta;
//                             bestNewParent = neigh;
//                             which_node = i+1;
//                         }
//                     }

//                     // comment_info 恢复topology所花费的时间
//                     utils::timer recover_timer;
//                     TreeNode::resetParent(node);
//                     TreeNode::setParent(node, originalParent);
//                     double recover_time = recover_timer.elapsed();
//                     time_stat.Inc_recover_topo(recover_time);
//                     // comment_info 恢复topology所花费的时间
//                 }

//                 tmp_steinerNode = nullptr;
//                 if (!exct_twice) {
//                     // comment_info 改变topology所花费的时间
//                     utils::timer topo_change_Timer;
//                     tmp_steinerNode = make_shared<TreeNode>(steinerPt);
//                     TreeNode::setParent(tmp_steinerNode, neighParent);
//                     TreeNode::resetParent(neigh);
//                     TreeNode::setParent(neigh, tmp_steinerNode);
//                     TreeNode::resetParent(node);
//                     TreeNode::setParent(node, tmp_steinerNode);
//                     tmp_steinerNode->id = nodes.size();
//                     double topo_change_time = topo_change_Timer.elapsed();
//                     time_stat.Inc_change_topo(topo_change_time);
//                     // comment_info 改变topology所花费的时间

//                     // comment_info 计算delay所花费的时间
//                     utils::timer delay_Timer;

//                     double sumDelay = 0;
//                     if (salt_delay) {
//                         // salt::ElmoreDelayEval delayEval(rd, tree);
//                         salt::ElmoreDelayEval delayEval(rd, tree, for_one_net_maxLB);
//                         sumDelay = delayEval.sumNorDelay;
//                     } else {
//                         sumDelay = time(_timing_engine, sta_net, tree, pin2loc);
//                     }

//                     double delay_time = delay_Timer.elapsed();
//                     time_stat.Inc_calc_delay(delay_time);
//                     // comment_info 计算delay所花费的时间

//                     if (sumDelay + 1e-12 < before_sumDelay) {
//                         auto delay_delta = sumDelay - before_sumDelay;
//                         if (delay_delta < best_delay_delta) {
//                             best_delay_delta = delay_delta;
//                             bestNewParent = neigh;
//                             which_node = 3;
//                         }
//                     }

//                     // comment_info 恢复topology所花费的时间
//                     utils::timer recover_timer;
//                     TreeNode::resetParent(tmp_steinerNode);
//                     TreeNode::resetParent(node);
//                     TreeNode::resetParent(neigh);
//                     TreeNode::setParent(node, originalParent);
//                     TreeNode::setParent(neigh, neighParent);
//                     double recover_time = recover_timer.elapsed();
//                     time_stat.Inc_recover_topo(recover_time);
//                     // comment_info 恢复topology所花费的时间
//                 }

//             }
//             if (bestNewParent) {
//                 assert(which_node != 0);
//                 candidateMoves.emplace_back(best_delay_delta, node, bestNewParent, which_node);
//                 // candidateMoves.emplace_back(bestWireLengthDelta, node, bestNewParent);
//             }
//             tmp_steinerNode = nullptr;
//             tmp_steinerNode.reset();
//         }
//         if (candidateMoves.empty()) {
//             break;
//         }
//         // cout << tree;


//         // Try candidate moves in the order of descending wire length savings
//         // Note that earlier moves may influence the legality of later one
//         sort(candidateMoves.begin(), candidateMoves.end(), [](const MoveT& lhs, const MoveT& rhs){
//             return get<0>(lhs) < get<0>(rhs);
//         });
//         for (const auto& move : candidateMoves) {
//             auto node = get<1>(move), neigh = get<2>(move);
//             auto which_node = get<3>(move);
//             auto neighParent = neigh->parent;
//             // check due to earlier moves
//             if (TreeNode::isAncestor(node, neighParent)) continue;

//             DTYPE pathLengthDelta =
//                 pathLengths[neighParent->id] + Dist(node->loc, neighParent->loc) - pathLengths[node->id];
//             if (pathLengthDelta > slacks[node->id]) continue;

//             auto steinerPt = GetNearestPoint(node, neigh);

//             // explain 预估一下timing
//             auto originalParent = node->parent;
//             std::shared_ptr<salt::TreeNode> tmp_steinerNode = nullptr;
//                 if (which_node == 1) {
//                     TreeNode::resetParent(node);
//                     TreeNode::setParent(node, neigh);
                    
//                     // pl不能违规
//                     DTYPE pathLengthDelta =
//                             pathLengths[neigh->id] + Dist(node->loc, neigh->loc) - pathLengths[node->id];
//                     if (pathLengthDelta > slacks[node->id]) continue;
                    
//                     // Connect(node, neigh);
//                 } else if (which_node == 2) {
//                     TreeNode::resetParent(node);
//                     TreeNode::setParent(node, neighParent);
//                     // Connect(node, neighParent);
//                 } else {
//                     tmp_steinerNode = make_shared<TreeNode>(steinerPt);
//                     TreeNode::setParent(tmp_steinerNode, neighParent);
//                     // Disconnect(neigh);
//                     TreeNode::resetParent(neigh);
//                     TreeNode::setParent(neigh, tmp_steinerNode);
//                     TreeNode::resetParent(node);
//                     TreeNode::setParent(node, tmp_steinerNode);
//                     // for later moves
//                     tmp_steinerNode->id = nodes.size();   
//                 }

//                 // cout << tree;

//                 double sumDelay = 0;
//                 if (salt_delay) {
//                     // salt::ElmoreDelayEval delayEval(rd, tree);
//                     salt::ElmoreDelayEval delayEval(rd, tree, for_one_net_maxLB);
//                     // auto sumDelay = 100;//delayEval.sumNorDelay;
//                     sumDelay = delayEval.sumNorDelay;
//                 } else {
//                     sumDelay = time(_timing_engine, sta_net, tree, pin2loc);
//                 }


//                 if (which_node == 1) {
//                     // TreeNode::setParent(node, neigh);
//                     TreeNode::resetParent(node);
//                     TreeNode::setParent(node, originalParent);
//                 } else if (which_node == 2) {
//                     // TreeNode::setParent(node, neighParent);
//                     TreeNode::resetParent(node);
//                     TreeNode::setParent(node, originalParent);
//                 } else {
//                     TreeNode::resetParent(tmp_steinerNode);
//                     TreeNode::resetParent(node);
//                     TreeNode::resetParent(neigh);
//                     TreeNode::setParent(node, originalParent);
//                     TreeNode::setParent(neigh, neighParent);
//                 }

//                 tmp_steinerNode.reset();

//                 if (sumDelay + 1e-12 < before_sumDelay) {
//                     before_sumDelay = sumDelay;
//                 } else {
//                     continue;
//                 }
//             // explain 预估一下timing

//             // break
//             Disconnect(node);
//             // reroot
//             if (which_node == 1) {
//                 Connect(node, neigh);
//             } else if (which_node == 2) {
//                 Connect(node, neighParent);
//             } else {
//                 auto steinerNode = make_shared<TreeNode>(steinerPt);
//                 Connect(steinerNode, neighParent);
//                 Disconnect(neigh);
//                 Connect(neigh, steinerNode);
//                 Connect(node, steinerNode);
//                 // for later moves
//                 steinerNode->id = nodes.size();
//                 // steinerNode->id = make_node_id++;
//                 nodes.push_back(steinerNode);
//                 pathLengths.push_back(pathLengths[neighParent->id] + steinerNode->WireToParent());
//                 slacks.push_back(Dist(steinerNode->loc, tree.source->loc) * (1 + eps) - pathLengths.back());
//             }
//             // update slack for later moves: first subtree, then path to source
//             TreeNode::preOrder(neighParent, UpdatePathLengths);
//             TreeNode::postOrder(neighParent, UpdateSlacks);

//             // update timing info
//             // salt::ElmoreDelayEval delayEval(rd, tree);
//             // auto updated_timing = delayEval.sumDelay;
//             tree.UpdateId();
//             // update timing info


//             auto tmp = neighParent;
//             while (tmp->parent) {
//                 slacks[tmp->parent->id] = min(slacks[tmp->parent->id], slacks[tmp->id]);
//                 tmp = tmp->parent;
//             }
//         }

//         // Finalize
//         tree.RemoveTopoRedundantSteiner();
//         // cout << tree;
//         tree.postOrderCopy([&](const shared_ptr<TreeNode>& node) {
//             // degree may change after post-order traversal of its children
//             if (node->pin) return;
//             if (node->children.empty()) {
//                 Disconnect(node);
//             } else if (node->children.size() == 1) {
//                 auto oldParent = node->parent, oldChild = node->children[0];
//                 Disconnect(node);
//                 Disconnect(oldChild);
//                 Connect(oldChild, oldParent);
//             }
//         });
//     }

//     // comment_info 输出运行时间情况
//     std::cout << "\033[1;33m";
//     cout << " Change topo " << time_stat.change_topo_cnt << " times, total run time " << time_stat.change_topo_time << endl;
//     cout << "Recover topo " << time_stat.recover_topo_cnt << " times, total run time " << time_stat.recover_topo_time << endl;
//     cout << "  Calc delay " << time_stat.calc_delay_cnt << " times, total run time " << time_stat.calc_delay_time << endl;
//     std::cout << "\033[0m";
// }

void DelayRefine::Substitute_delay_aware(Tree& tree, double eps, double rd, bool useRTree) {
    bgi::rtree<RNode, bgi::rstar<8>, bgi::indexable<RNode>, RNodeComp> rtree;
    if (useRTree) {
        tree.postOrder([&](const shared_ptr<TreeNode>& n) {
            if (n->parent) {
                BBox s;
                bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(n->parent->loc.x, n->parent->loc.y)), s);
                rtree.insert({s, n});
                // std::cout << "Bounding Box: " << bg::dsv(s) << std::endl;
            }
        });
    }
    auto Disconnect = [&](const shared_ptr<TreeNode>& n) {
        if (useRTree) {
            BBox s;
            bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(n->parent->loc.x, n->parent->loc.y)), s);
            rtree.remove({s, n});
        }
        TreeNode::resetParent(n);
    };
    auto Connect = [&](const shared_ptr<TreeNode>& n, const shared_ptr<TreeNode>& parent) {
        TreeNode::setParent(n, parent);
        if (useRTree) {
            BBox s;
            bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(parent->loc.x, parent->loc.y)), s);
            rtree.insert({s, n});
        }
    };

    auto make_node_id = 10*tree.UpdateId();

    auto outchild = [&](const shared_ptr<TreeNode>& node) {
        if (node->children.empty()) {
            cout << "Node " << node->id << " has no children" << endl;
        } else {
            cout << "Node " << node->id << " has children: ";
            for (auto c : node->children) {
                cout << c->id << " ";
            }
            cout << endl;
        }
    };
    // tree.preOrder(outchild);

    // cout << tree;
    // std::shared_ptr<salt::TreeNode> tmp_steinerNode = nullptr;
        // 预备
    salt::ElmoreDelayEval before_delayEval(rd, tree);
    auto before_sumDelay = before_delayEval.sumNorDelay;

    auto for_one_net_maxLB = before_delayEval._maxLb;

    int iter_num = 0;

    // comment_info 统计改变topology和计算delay所用的时间
    TimeStat time_stat;

    while (true) {
    iter_num++;
        // Get nearest neighbors
        int num = tree.UpdateId();
        cout << "num: " << num << endl;
        vector<shared_ptr<TreeNode>> nodes = tree.ObtainNodes(),
                                     orderedNodes(nodes.size());  // note: all pins should be covered
        if (nodes.size() != num) {
            cout << "debug\n";
        }
        vector<Point> points(nodes.size());
        for (int i = 0; i < nodes.size(); ++i) {
            // cout << "node " << nodes[i]->id << endl;
            orderedNodes[nodes[i]->id] = nodes[i];
            points[nodes[i]->id] = nodes[i]->loc;  // TODO: move within bracket
        }
        nodes = orderedNodes;
        vector<vector<int>> nearestNeighbors;
        if (!useRTree) {
            MstBuilder mstB;
            mstB.GetAllNearestNeighbors(points, nearestNeighbors);
        } else {
            nearestNeighbors.resize(nodes.size());
            for (auto n : nodes) {
                if (n->parent) {
                    Point c = n->loc;  // center
                    DTYPE radius = n->WireToParent();
                    // diamond is too slow...
                    // BPolygon diamond;
                    // diamond.outer().emplace_back(c.x - radius, c.y);
                    // diamond.outer().emplace_back(c.x, c.y + radius);
                    // diamond.outer().emplace_back(c.x + radius, c.y);
                    // diamond.outer().emplace_back(c.x, c.y - radius);
                    // diamond.outer().emplace_back(c.x - radius, c.y);
                    int tora = 10;
                    BBox queryBox{{c.x - tora*radius, c.y - tora*radius}, {c.x + tora*radius, c.y + tora*radius}};
                    vector<RNode> cands;
                    rtree.query(bgi::intersects(queryBox), back_inserter(cands));  // TODO: change back_inserter
                    for (const auto& cand : cands) {
                        nearestNeighbors[n->id].push_back(cand.second->id);
                    }
                }
            }
        }

        // Prune descendants in nearest neighbors
        vector<int> preOrderIdxes(nodes.size(), -1);
        int globalPreOrderIdx = 0;
        function<void(const shared_ptr<TreeNode>&)> removeDescendants = [&](const shared_ptr<TreeNode>& node) {
            preOrderIdxes[node->id] = globalPreOrderIdx++;
            for (auto child : node->children) {
                removeDescendants(child);
            }
            for (auto& neighIdx : nearestNeighbors[node->id]) {
                int neighPreOrderIdx = preOrderIdxes[neighIdx];
                if (neighPreOrderIdx != -1 && neighPreOrderIdx >= preOrderIdxes[node->id]) {
                    neighIdx = -1;  // -1 stands for "descendant"
                }
            }
        };
        removeDescendants(tree.source);

        // Init path lengths and subtree slacks
        vector<DTYPE> pathLengths(nodes.size());
        vector<DTYPE> slacks(nodes.size());
        auto UpdatePathLengths = [&](const shared_ptr<TreeNode>& node) {
            if (node->parent) {
                pathLengths[node->id] = pathLengths[node->parent->id] + node->WireToParent();
            } else {
                pathLengths[node->id] = 0;
            }
        };
        auto UpdateSlacks = [&](const shared_ptr<TreeNode>& node) {
            if (node->children.empty()) {
                slacks[node->id] =
                    Dist(node->loc, tree.source->loc) * (1 + eps) - pathLengths[node->id];  // floor here...
            } else {
                DTYPE minSlack = Dist(node->loc, tree.source->loc) * (1 + eps) - pathLengths[node->id];
                for (auto child : node->children) {
                    minSlack = min(minSlack, slacks[child->id]);
                }
                slacks[node->id] = minSlack;
            }
        };
        tree.preOrder(UpdatePathLengths);
        tree.postOrder(UpdateSlacks);

        // Find legal candidate moves
        // using MoveT = tuple<DTYPE, shared_ptr<TreeNode>, shared_ptr<TreeNode>>;
        using MoveT = tuple<double, shared_ptr<TreeNode>, shared_ptr<TreeNode>>;
        vector<MoveT> candidateMoves;  // <wireLengthDelta, node, newParent>
        auto GetNearestPoint = [](const shared_ptr<TreeNode>& target, const shared_ptr<TreeNode>& neigh) {
            Box box(neigh->loc, neigh->parent->loc);
            box.Legalize();
            return box.GetNearestPointTo(target->loc);
        };




        vector<double> cap(num, 0);
        tree.postOrder([&](const shared_ptr<TreeNode>& node) {
            if (node->pin && node != tree.source) cap[node->id] = node->pin->cap;
            for (auto c : node->children) {
                cap[node->id] += cap[c->id];
                cap[node->id] += c->WireToParent() * 8e-20;
            }
        });

        for (auto node : nodes) {
            std::shared_ptr<salt::TreeNode> tmp_steinerNode = nullptr;
            if (!(node->parent)) {
                continue;
            }

            auto originalParent = node->parent;
            DTYPE bestWireLengthDelta = 0;  // the negative, the better

            double best_delay_delta = 0;

            shared_ptr<TreeNode> bestNewParent;
            for (int neighIdx : nearestNeighbors[node->id]) {
                if (node->id == 2 && neighIdx == 41) {
                    cout << "afadsfn";
                }
                // bug   why this?
                if (neighIdx >= nodes.size()) continue;
                // bug   why this?
                if (neighIdx == -1 || !nodes[neighIdx]->parent) continue;
                auto neigh = nodes[neighIdx];
                auto neighParent = neigh->parent;

                // bug   why this?
                if (node == neighParent) continue;
                if (node == neigh) continue;
                // bug   why this?
                auto steinerPt = GetNearestPoint(node, neigh);
                DTYPE wireLengthDelta = Dist(node->loc, steinerPt) - node->WireToParent();
                // if (wireLengthDelta < bestWireLengthDelta) {  // has wire length improvement
                //     DTYPE pathLengthDelta =
                //         pathLengths[neighParent->id] + Dist(node->loc, neighParent->loc) - pathLengths[node->id];
                //     if (pathLengthDelta <= slacks[node->id]) {  // make path length under control
                //         bestWireLengthDelta = wireLengthDelta;
                //         bestNewParent = neigh;
                //     }
                // }

                // pl不能违规
                DTYPE pathLengthDelta =
                        pathLengths[neighParent->id] + Dist(node->loc, neighParent->loc) - pathLengths[node->id];
                if (pathLengthDelta > slacks[node->id]) continue;

                // cout << tree;
                tmp_steinerNode = nullptr;
                // comment_info 改变topology所花费的时间
                utils::timer topo_change_Timer;
                if (steinerPt == neigh->loc) {
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, neigh);
                    // Connect(node, neigh);
                } else if (steinerPt == neighParent->loc) {
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, neighParent);
                    // Connect(node, neighParent);
                } else {
                    tmp_steinerNode = make_shared<TreeNode>(steinerPt);
                    TreeNode::setParent(tmp_steinerNode, neighParent);
                    // Disconnect(neigh);
                    TreeNode::resetParent(neigh);
                    TreeNode::setParent(neigh, tmp_steinerNode);
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, tmp_steinerNode);
                    // for later moves
                    tmp_steinerNode->id = nodes.size();
                    // tmp_steinerNode->id = make_node_id++;
                    // nodes.push_back(steinerNode);
                    // pathLengths.push_back(pathLengths[neighParent->id] + steinerNode->WireToParent());
                    // slacks.push_back(Dist(steinerNode->loc, tree.source->loc) * (1 + eps) - pathLengths.back());
                    
                }

                // comment_info 改变topology所花费的时间
                double topo_change_time = topo_change_Timer.elapsed();
                time_stat.Inc_change_topo(topo_change_time);

                // cout << tree;

                // comment_info 计算delay所花费的时间
                utils::timer delay_Timer;
                // salt::ElmoreDelayEval delayEval(rd, tree);
                salt::ElmoreDelayEval delayEval(rd, tree, for_one_net_maxLB);
                // auto sumDelay = 100;//delayEval.sumNorDelay;
                auto sumDelay = delayEval.sumNorDelay;
                // comment_info 计算delay所花费的时间
                double delay_time = delay_Timer.elapsed();
                time_stat.Inc_calc_delay(delay_time);

                if (sumDelay + 1e-12 < before_sumDelay) {
                    auto delay_delta = sumDelay - before_sumDelay;
                    if (delay_delta < best_delay_delta) {
                        best_delay_delta = delay_delta;
                        bestNewParent = neigh;
                    }
                }

                // comment_info 恢复topology所花费的时间
                utils::timer recover_timer;
                if (steinerPt == neigh->loc) {
                    // TreeNode::setParent(node, neigh);
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, originalParent);
                } else if (steinerPt == neighParent->loc) {
                    // TreeNode::setParent(node, neighParent);
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, originalParent);
                } else {
                    TreeNode::resetParent(tmp_steinerNode);
                    TreeNode::resetParent(node);
                    TreeNode::resetParent(neigh);
                    TreeNode::setParent(node, originalParent);
                    TreeNode::setParent(neigh, neighParent);
                }

                // comment_info 恢复topology所花费的时间
                double recover_time = recover_timer.elapsed();
                time_stat.Inc_recover_topo(recover_time);

                // cout << tree;
                // cout << endl;
                if (bestNewParent) {
                    if (node->id == 14 && bestNewParent->id == 1) {
                    cout << "afadsfn";
                    }
                    if (node->id == 7 && bestNewParent->id == 14) {
                        cout << "afadsfn";
                    }
                    if (node->id == 5 && bestNewParent->id == 8) {
                        cout << "afadsfn";
                    }
                }

            }
            if (bestNewParent) {
                candidateMoves.emplace_back(best_delay_delta, node, bestNewParent);
                // candidateMoves.emplace_back(bestWireLengthDelta, node, bestNewParent);
            }
            tmp_steinerNode = nullptr;
            tmp_steinerNode.reset();
        }
        if (candidateMoves.empty()) {
            break;
        }
        // cout << tree;


        // Try candidate moves in the order of descending wire length savings
        // Note that earlier moves may influence the legality of later one
        sort(candidateMoves.begin(), candidateMoves.end(), [](const MoveT& lhs, const MoveT& rhs){
            return get<0>(lhs) < get<0>(rhs);
        });
        for (const auto& move : candidateMoves) {
            auto node = get<1>(move), neigh = get<2>(move);
            auto neighParent = neigh->parent;
            // check due to earlier moves
            if (TreeNode::isAncestor(node, neighParent)) continue;
            DTYPE pathLengthDelta =
                pathLengths[neighParent->id] + Dist(node->loc, neighParent->loc) - pathLengths[node->id];
            if (pathLengthDelta > slacks[node->id]) continue;
            auto steinerPt = GetNearestPoint(node, neigh);
            DTYPE wireLengthDelta = Dist(node->loc, steinerPt) - node->WireToParent();
            // if (wireLengthDelta >= 0) continue;


            // explain 预估一下timing
            auto originalParent = node->parent;
            std::shared_ptr<salt::TreeNode> tmp_steinerNode = nullptr;
                if (steinerPt == neigh->loc) {
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, neigh);
                    // Connect(node, neigh);
                } else if (steinerPt == neighParent->loc) {
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, neighParent);
                    // Connect(node, neighParent);
                } else {
                    tmp_steinerNode = make_shared<TreeNode>(steinerPt);
                    TreeNode::setParent(tmp_steinerNode, neighParent);
                    // Disconnect(neigh);
                    TreeNode::resetParent(neigh);
                    TreeNode::setParent(neigh, tmp_steinerNode);
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, tmp_steinerNode);
                    // for later moves
                    tmp_steinerNode->id = nodes.size();   
                }

                // cout << tree;

                // salt::ElmoreDelayEval delayEval(rd, tree);
                salt::ElmoreDelayEval delayEval(rd, tree, for_one_net_maxLB);
                // auto sumDelay = 100;//delayEval.sumNorDelay;
                auto sumDelay = delayEval.sumNorDelay;


                if (steinerPt == neigh->loc) {
                    // TreeNode::setParent(node, neigh);
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, originalParent);
                } else if (steinerPt == neighParent->loc) {
                    // TreeNode::setParent(node, neighParent);
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, originalParent);
                } else {
                    TreeNode::resetParent(tmp_steinerNode);
                    TreeNode::resetParent(node);
                    TreeNode::resetParent(neigh);
                    TreeNode::setParent(node, originalParent);
                    TreeNode::setParent(neigh, neighParent);
                }

                // if (steinerPt == neigh->loc) {
                //     // TreeNode::setParent(node, neigh);
                //     TreeNode::resetParent(node);
                //     TreeNode::setParent(node, neighParent);
                // } else if (steinerPt == neighParent->loc) {
                //     // TreeNode::setParent(node, neighParent);
                //     TreeNode::resetParent(node);
                //     TreeNode::setParent(node, neighParent);
                // } else {
                //     TreeNode::resetParent(tmp_steinerNode);
                //     TreeNode::resetParent(node);
                //     TreeNode::resetParent(neigh);
                //     TreeNode::setParent(node, neighParent);
                //     TreeNode::setParent(neigh, tmp_steinerNode);
                // }

                tmp_steinerNode.reset();

                if (sumDelay + 1e-12 < before_sumDelay) {
                    before_sumDelay = sumDelay;
                } else {
                    continue;
                }
            // explain 预估一下timing

            // cout << tree;


            // break
            Disconnect(node);
            // reroot
            if (steinerPt == neigh->loc) {
                Connect(node, neigh);
            } else if (steinerPt == neighParent->loc) {
                Connect(node, neighParent);
            } else {
                auto steinerNode = make_shared<TreeNode>(steinerPt);
                Connect(steinerNode, neighParent);
                Disconnect(neigh);
                Connect(neigh, steinerNode);
                Connect(node, steinerNode);
                // for later moves
                steinerNode->id = nodes.size();
                // steinerNode->id = make_node_id++;
                nodes.push_back(steinerNode);
                pathLengths.push_back(pathLengths[neighParent->id] + steinerNode->WireToParent());
                slacks.push_back(Dist(steinerNode->loc, tree.source->loc) * (1 + eps) - pathLengths.back());
            }
            // update slack for later moves: first subtree, then path to source
            TreeNode::preOrder(neighParent, UpdatePathLengths);
            TreeNode::postOrder(neighParent, UpdateSlacks);

            // update timing info
            // salt::ElmoreDelayEval delayEval(rd, tree);
            // auto updated_timing = delayEval.sumDelay;
            tree.UpdateId();
            // update timing info


            auto tmp = neighParent;
            while (tmp->parent) {
                slacks[tmp->parent->id] = min(slacks[tmp->parent->id], slacks[tmp->id]);
                tmp = tmp->parent;
            }
        }

        // Finalize
        tree.RemoveTopoRedundantSteiner();
        // cout << tree;
        tree.postOrderCopy([&](const shared_ptr<TreeNode>& node) {
            // degree may change after post-order traversal of its children
            if (node->pin) return;
            if (node->children.empty()) {
                Disconnect(node);
            } else if (node->children.size() == 1) {
                auto oldParent = node->parent, oldChild = node->children[0];
                Disconnect(node);
                Disconnect(oldChild);
                Connect(oldChild, oldParent);
            }
        });
    }

    // comment_info 输出运行时间情况
    std::cout << "\033[1;33m";
    cout << " Change topo " << time_stat.change_topo_cnt << " times, total run time " << time_stat.change_topo_time << endl;
    cout << "Recover topo " << time_stat.recover_topo_cnt << " times, total run time " << time_stat.recover_topo_time << endl;
    cout << "  Calc delay " << time_stat.calc_delay_cnt << " times, total run time " << time_stat.calc_delay_time << endl;
    std::cout << "\033[0m";
}


// ==================================================================================
// namespace bg = boost::geometry;
// namespace bgi = boost::geometry::index;

// using BPoint = bg::model::point<DTYPE, 2, bg::cs::cartesian>;
// using BSegment = bg::model::segment<BPoint>;
// using BBox = bg::model::box<BPoint>;
// using BPolygon = bg::model::polygon<BPoint>;
// using RNode = pair<BBox, shared_ptr<TreeNode>>;  // R-Tree node
// struct RNodeComp {
//     bool operator()(const RNode& l, const RNode& r) const {
//         return bg::equals(l.first, r.first) && l.second == r.second;
//     }
// };


shared_ptr<TreeNode> DelayRefine::findLowestCommonAncestor(shared_ptr<TreeNode> p, shared_ptr<TreeNode> q) {
    if (p == nullptr || q == nullptr)
        return nullptr;

    // 创建一个哈希集合，用于存储p和其所有祖先节点
    unordered_set<shared_ptr<TreeNode>> ancestors;

    // 遍历节点p及其所有祖先，并加入哈希集合
    while (p != nullptr) {
        ancestors.insert(p);
        p = p->parent;
    }

    // 遍历节点q及其所有祖先，找到第一个出现在哈希集合中的节点，即为最近公共祖先
    while (q != nullptr) {
        if (ancestors.count(q) > 0)
            return q;
        q = q->parent;
    }

    return nullptr; // TODO 没有最近公共祖先 error!!! 至少根节点是公共祖先
}

// 枚举类
enum class Which_Case { a1, a2, b1, b2, c };
// p=1, q=3
double DelayRefine::calc_delay_delta(shared_ptr<TreeNode> p,
                        shared_ptr<TreeNode> q,
                        shared_ptr<TreeNode> critical,
                        shared_ptr<TreeNode> node,
                        shared_ptr<TreeNode> neighbor,
                        Point steinerPt_loc,
                        vector<double> node_cap,
                        vector<DTYPE> pathLengths,
                        DTYPE wireLengthDelta) {
    // if (p == nullptr || q == nullptr) return nullptr;

    // 创建一个哈希集合，用于存储p和其所有祖先节点
    unordered_set<shared_ptr<TreeNode>> p_ancestors;
    unordered_set<shared_ptr<TreeNode>> q_ancestors;

    // 遍历节点p及其所有祖先，并加入哈希集合
    while (p != nullptr) {
        p_ancestors.insert(p);
        p = p->parent;
    }

    shared_ptr<TreeNode> p_q_ancestor = nullptr;

    // 遍历节点q及其所有祖先，找到第一个出现在哈希集合中的节点，即为p，q的最近公共祖先
    while (q != nullptr) {
        q_ancestors.insert(q);
        if (p_ancestors.count(q) > 0) {
            p_q_ancestor = q;
            break;
        }
        q = q->parent;
    }

    // TODO 重置p_ancestors
    p_ancestors.clear();
    while (p != nullptr) {
        p_ancestors.insert(p);
        p = p->parent;
        if (p == p_q_ancestor) {
            break;
        }
    }

    Which_Case which_case;
    shared_ptr<TreeNode> critical_neighParent = nullptr;
    shared_ptr<TreeNode> critical_p = nullptr;

    auto critical_tmp = critical;

    while (critical != nullptr) {
        if (critical == p_q_ancestor) {
            which_case = Which_Case::c;
            break;
        }

        if (q_ancestors.count(critical) > 0) {
            if (critical != neighbor) {
                which_case = Which_Case::a1;
                critical_neighParent = critical;
                break;
            } else {
                which_case = Which_Case::a2;
                break;
            }
        } else if (p_ancestors.count(critical) > 0) {
            if (critical != node) {
                which_case = Which_Case::b1;
                critical_p = critical;
                break;
            } else {
                which_case = Which_Case::b2;
                break;
            }
        } else {
            critical = critical->parent;
        }
    }

    if (!critical) {
        which_case = Which_Case::c;
    }

    /**
     * base info
     */
    auto rd = 25.35;  // TODO  get rd
    auto unitCap = salt::ElmoreDelayEval::unitCap;
    auto unitRes = salt::ElmoreDelayEval::unitRes;

    double delay_delta = 0;

    switch (which_case) {
        case Which_Case::a1: {
            auto root_2_p_q_ancestor_len = pathLengths[p_q_ancestor->id];
            auto p_q_ancestor_2_critical_neighParent =
                pathLengths[critical_neighParent->id] - pathLengths[p_q_ancestor->id];
            auto node_load_cap = node_cap[node->id];
            auto added_wire_cap = Dist(node->loc, steinerPt_loc) * unitCap;

            delay_delta = rd * wireLengthDelta * unitCap + root_2_p_q_ancestor_len * unitRes * wireLengthDelta * unitCap-
                          p_q_ancestor_2_critical_neighParent * unitRes * (node_load_cap + added_wire_cap);
            break;
        }

        case Which_Case::a2: {
            auto root_2_p_q_ancestor_len = pathLengths[p_q_ancestor->id];
            auto node_load_cap = node_cap[node->id];

            auto pl_3_5 = Dist(q->loc, steinerPt_loc);
            auto pl_2_5 = Dist(node->loc, steinerPt_loc);
            auto pl_4_5 = Dist(neighbor->loc, steinerPt_loc);

            auto delta = -pl_3_5 * unitRes * (node_load_cap + (pl_2_5 + pl_4_5) * unitCap);

            delay_delta = rd * wireLengthDelta * unitCap + root_2_p_q_ancestor_len * unitRes * wireLengthDelta * unitCap + delta;
        }

        case Which_Case::b1: {
            auto root_2_p_q_ancestor_len = pathLengths[p_q_ancestor->id];
            auto p_q_ancestor_2_critical_p = pathLengths[critical_p->id] - pathLengths[p_q_ancestor->id];
            auto node_load_cap = node_cap[node->id];

            delay_delta = rd * wireLengthDelta * unitCap + root_2_p_q_ancestor_len * unitRes * wireLengthDelta * unitCap +
                          p_q_ancestor_2_critical_p * unitRes * node_load_cap;
        }

        case Which_Case::b2: {
            //  TODO TODO
            delay_delta = 10;
        }

        case Which_Case::c: {
            auto ancestor = DelayRefine::findLowestCommonAncestor(critical_tmp, p_q_ancestor);
            auto pl = pathLengths[ancestor->id];

            delay_delta = rd * wireLengthDelta * unitCap + pl * unitRes * wireLengthDelta * unitCap;
        }
        default:
            break;
    }
    return delay_delta;
}

void DelayRefine::Substitute_critical_delay_aware(Tree& tree, double eps, double rd, bool useRTree) {
    bgi::rtree<RNode, bgi::rstar<8>, bgi::indexable<RNode>, RNodeComp> rtree;
    if (useRTree) {
        tree.postOrder([&](const shared_ptr<TreeNode>& n) {
            if (n->parent) {
                BBox s;
                bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(n->parent->loc.x, n->parent->loc.y)), s);
                rtree.insert({s, n});
            }
        });
    }
    auto Disconnect = [&](const shared_ptr<TreeNode>& n) {
        if (useRTree) {
            BBox s;
            bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(n->parent->loc.x, n->parent->loc.y)), s);
            rtree.remove({s, n});
        }
        TreeNode::resetParent(n);
    };
    auto Connect = [&](const shared_ptr<TreeNode>& n, const shared_ptr<TreeNode>& parent) {
        TreeNode::setParent(n, parent);
        if (useRTree) {
            BBox s;
            bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(parent->loc.x, parent->loc.y)), s);
            rtree.insert({s, n});
        }
    };
    cout <<tree;

    int max_iter = tree.UpdateId()-1;
    int iter = 0;
    while (true) {
        if (iter > max_iter) break;
        iter++;
        // Get nearest neighbors
        int num = tree.UpdateId();
        vector<shared_ptr<TreeNode>> nodes = tree.ObtainNodes(),
                                     orderedNodes(nodes.size());  // note: all pins should be covered
        vector<Point> points(nodes.size());
        for (int i = 0; i < nodes.size(); ++i) {
            orderedNodes[nodes[i]->id] = nodes[i];
            points[nodes[i]->id] = nodes[i]->loc;  // TODO: move within bracket
        }
        nodes = orderedNodes;
        vector<vector<int>> nearestNeighbors;
        if (!useRTree) {
            MstBuilder mstB;
            mstB.GetAllNearestNeighbors(points, nearestNeighbors);
        } else {
            nearestNeighbors.resize(nodes.size());
            for (auto n : nodes) {
                if (n->parent) {
                    Point c = n->loc;  // center
                    DTYPE radius = n->WireToParent();
                    // diamond is too slow...
                    // BPolygon diamond;
                    // diamond.outer().emplace_back(c.x - radius, c.y);
                    // diamond.outer().emplace_back(c.x, c.y + radius);
                    // diamond.outer().emplace_back(c.x + radius, c.y);
                    // diamond.outer().emplace_back(c.x, c.y - radius);
                    // diamond.outer().emplace_back(c.x - radius, c.y);
                    int to = 4;
                    BBox queryBox{{c.x - to*radius, c.y - to*radius}, {c.x + to*radius, c.y + to*radius}};
                    vector<RNode> cands;
                    rtree.query(bgi::intersects(queryBox), back_inserter(cands));  // TODO: change back_inserter
                    for (const auto& cand : cands) {
                        nearestNeighbors[n->id].push_back(cand.second->id);
                    }
                }
            }
        }

        // Prune descendants in nearest neighbors
        vector<int> preOrderIdxes(nodes.size(), -1);
        int globalPreOrderIdx = 0;
        function<void(const shared_ptr<TreeNode>&)> removeDescendants = [&](const shared_ptr<TreeNode>& node) {
            preOrderIdxes[node->id] = globalPreOrderIdx++;
            for (auto child : node->children) {
                removeDescendants(child);
            }
            for (auto& neighIdx : nearestNeighbors[node->id]) {
                int neighPreOrderIdx = preOrderIdxes[neighIdx];
                if (neighPreOrderIdx != -1 && neighPreOrderIdx >= preOrderIdxes[node->id]) {
                    neighIdx = -1;  // -1 stands for "descendant"
                }
            }
        };
        removeDescendants(tree.source);

        salt::ElmoreDelayEval before_delayEval(rd, tree);
        auto max_delay_node_id = before_delayEval.max_delay_node_id;
        auto unitCap = salt::ElmoreDelayEval::unitCap;
        auto unitRes = salt::ElmoreDelayEval::unitRes;
        // 容忍度

    // 将科学计数法表示的数字转换为字符串
    std::stringstream ss;
    ss << std::scientific << std::setprecision(15) << before_delayEval.maxDelay;
    std::string numberString = ss.str();
    // 找到第一个数值
    char firstDigit = '0';
    for (char c : numberString) {
        if (std::isdigit(c)) {
            firstDigit = c;
            break;
        }
    }
    // char 转 double
    int firstDigitInt = firstDigit - '0';
    if (firstDigitInt > 5) firstDigitInt = 1;
        // auto tolerance = before_delayEval.maxDelay * 1e-2 / 1;
        // cout << "firstDigitInt " << firstDigitInt << endl;
        auto tolerance = before_delayEval.maxDelay * 1e-2 / double(firstDigitInt);
        // cout << " tolerance " << tolerance << endl;


        // Init path lengths and subtree slacks
        vector<DTYPE> pathLengths(nodes.size());
        vector<DTYPE> slacks(nodes.size());

        vector<int> node_levels(nodes.size());
        vector<double> node_cap(nodes.size(), 0);

        auto UpdatePathLengths = [&](const shared_ptr<TreeNode>& node) {
            if (node->parent) {
                pathLengths[node->id] = pathLengths[node->parent->id] + node->WireToParent();
            } else {
                pathLengths[node->id] = 0;
            }
        };
        auto UpdateSlacks = [&](const shared_ptr<TreeNode>& node) {
            if (node->children.empty()) {
                slacks[node->id] =
                    Dist(node->loc, tree.source->loc) * (1 + eps) - pathLengths[node->id];  // floor here...
            } else {
                DTYPE minSlack = Dist(node->loc, tree.source->loc) * (1 + eps) - pathLengths[node->id];
                for (auto child : node->children) {
                    minSlack = min(minSlack, slacks[child->id]);
                }
                slacks[node->id] = minSlack;
            }
        };
        tree.preOrder(UpdatePathLengths);
        tree.postOrder(UpdateSlacks);

        // explain 记录节点负载电容的信息
        tree.postOrder([&](const shared_ptr<TreeNode>& node) {
        if (node->pin && node != tree.source) node_cap[node->id] = node->pin->cap;
        for (auto c : node->children) {
            node_cap[node->id] += node_cap[c->id];
            node_cap[node->id] += c->WireToParent() * unitCap;
            }
        });

        // Find legal candidate moves
        // using MoveT = tuple<DTYPE, shared_ptr<TreeNode>, shared_ptr<TreeNode>>;
        using MoveT = tuple<double, shared_ptr<TreeNode>, shared_ptr<TreeNode>>;
        vector<MoveT> candidateMoves;  // <wireLengthDelta, node, newParent>
        auto GetNearestPoint = [](const shared_ptr<TreeNode>& target, const shared_ptr<TreeNode>& neigh) {
            Box box(neigh->loc, neigh->parent->loc);
            box.Legalize();
            return box.GetNearestPointTo(target->loc);
        };
        for (auto node : nodes) {
            if (!(node->parent)) {
                continue;
            }
            DTYPE bestWireLengthDelta = 0;  // the negative, the better
            double best_delay_delta = 0;  // the negative, the better
            shared_ptr<TreeNode> bestNewParent;
            for (int neighIdx : nearestNeighbors[node->id]) {
                // if (node->id == 14 && neighIdx == 1) {
                //     cout << "afadsfn";
                // }
                // if (node->id == 7 && neighIdx == 14) {
                //     cout << "afadsfn";
                // }
                // if (node->id == 5 && neighIdx == 8) {
                //     cout << "afadsfn";
                // }
                if (neighIdx == -1 || !nodes[neighIdx]->parent) continue;
                auto neigh = nodes[neighIdx];
                auto neighParent = neigh->parent;
                auto steinerPt = GetNearestPoint(node, neigh);
                DTYPE wireLengthDelta = Dist(node->loc, steinerPt) - node->WireToParent();

                // pl不能违规
                DTYPE pathLengthDelta =
                        pathLengths[neighParent->id] + Dist(node->loc, neighParent->loc) - pathLengths[node->id];
                if (pathLengthDelta > slacks[node->id]) continue;

                // auto lowest_common_ancestor = findLowestCommonAncestor(node->parent, neighParent);
                // auto common_path_length = pathLengths[lowest_common_ancestor->id];

                // auto extra_cap = node_cap[node->id] + Dist(node->loc, steinerPt) * unitCap;

                // // auto delay_delta = wireLengthDelta*(rd + common_path_length * unitRes) + 
                // auto source_delay_delta = rd * wireLengthDelta;
                // auto wire_delay_delta_1 = common_path_length * unitRes * wireLengthDelta;
                // auto wire_delay_delta_2 = (pathLengths[neighParent->id] - common_path_length) * unitRes * extra_cap;
                // auto delay_delta = source_delay_delta + wire_delay_delta_1 + wire_delay_delta_2;

                // comment_info 计算critical node的delay delta
                auto critical_node = nodes[max_delay_node_id];
                auto delay_delta = DelayRefine::calc_delay_delta(node->parent,
                                                    neighParent,
                                                    critical_node,
                                                    node,
                                                    neigh,
                                                    steinerPt,
                                                    node_cap,
                                                    pathLengths,
                                                    wireLengthDelta);
                


                // // comment_info 如果导致max delay node的level增加了，就continue
                // if (TreeNode::IsAncestor(neighParent, nodes[max_delay_node_id]) && wireLengthDelta > -1000) {
                //     continue;
                // }

                // if (wireLengthDelta < bestWireLengthDelta) {  // has wire length improvement
                //     DTYPE pathLengthDelta =
                //         pathLengths[neighParent->id] + Dist(node->loc, neighParent->loc) - pathLengths[node->id];
                //     if (pathLengthDelta <= slacks[node->id]) {  // make path length under control
                //         bestWireLengthDelta = wireLengthDelta;
                //         bestNewParent = neigh;
                //     }
                // }
                if (delay_delta >= 0) continue;
                if (delay_delta + tolerance < best_delay_delta) {  // has wire length improvement
                // if (delay_delta + 1e-18 < best_delay_delta) {  // has wire length improvement
                    DTYPE pathLengthDelta =
                        pathLengths[neighParent->id] + Dist(node->loc, neighParent->loc) - pathLengths[node->id];
                    if (pathLengthDelta <= slacks[node->id]) {  // make path length under control
                        best_delay_delta = delay_delta;
                        bestNewParent = neigh;
                    }
                }

                // if (bestNewParent) {
                //     if (node->id == 14 && bestNewParent->id == 1) {
                //     cout << "afadsfn";
                //     }
                //     if (node->id == 7 && bestNewParent->id == 14) {
                //         cout << "afadsfn";
                //     }
                //     if (node->id == 5 && bestNewParent->id == 8) {
                //         cout << "afadsfn";
                //     }
                // }
            }
            if (bestNewParent) {
                candidateMoves.emplace_back(best_delay_delta, node, bestNewParent);
            }
        }
        if (candidateMoves.empty()) {
            break;
        }

        // Try candidate moves in the order of descending wire length savings
        // Note that earlier moves may influence the legality of later one
        sort(candidateMoves.begin(), candidateMoves.end(), [](const MoveT& lhs, const MoveT& rhs){
            return get<0>(lhs) < get<0>(rhs);
        });
        for (const auto& move : candidateMoves) {
            auto node = get<1>(move), neigh = get<2>(move);
            auto neighParent = neigh->parent;
            // check due to earlier moves
            if (TreeNode::isAncestor(node, neighParent)) continue;
            DTYPE pathLengthDelta =
                pathLengths[neighParent->id] + Dist(node->loc, neighParent->loc) - pathLengths[node->id];
            // if (pathLengthDelta > slacks[node->id]) continue;
            auto steinerPt = GetNearestPoint(node, neigh);
            DTYPE wireLengthDelta = Dist(node->loc, steinerPt) - node->WireToParent();
            // if (wireLengthDelta >= 0) continue;


            // comment_info 计算critical node的delay delta
            auto critical_node = nodes[max_delay_node_id];
            auto delay_delta = DelayRefine::calc_delay_delta(node->parent,
                                                neighParent,
                                                critical_node,
                                                node,
                                                neigh,
                                                steinerPt,
                                                node_cap,
                                                pathLengths,
                                                wireLengthDelta);
            if (delay_delta >= 0) {
                continue;
            }
            // comment_info 


            // break
            Disconnect(node);
            // reroot
            if (steinerPt == neigh->loc) {
                Connect(node, neigh);
            } else if (steinerPt == neighParent->loc) {
                Connect(node, neighParent);
            } else {
                auto steinerNode = make_shared<TreeNode>(steinerPt);
                Connect(steinerNode, neighParent);
                Disconnect(neigh);
                Connect(neigh, steinerNode);
                Connect(node, steinerNode);
                // for later moves
                steinerNode->id = nodes.size();
                nodes.push_back(steinerNode);
                pathLengths.push_back(pathLengths[neighParent->id] + steinerNode->WireToParent());
                slacks.push_back(Dist(steinerNode->loc, tree.source->loc) * (1 + eps) - pathLengths.back());
            }
            // update slack for later moves: first subtree, then path to source
            TreeNode::preOrder(neighParent, UpdatePathLengths);
            TreeNode::postOrder(neighParent, UpdateSlacks);
            auto tmp = neighParent;
            while (tmp->parent) {
                slacks[tmp->parent->id] = min(slacks[tmp->parent->id], slacks[tmp->id]);
                tmp = tmp->parent;
            }
        }

        // Finalize
        // tree.RemoveTopoRedundantSteiner();
        tree.postOrderCopy([&](const shared_ptr<TreeNode>& node) {
            // degree may change after post-order traversal of its children
            if (node->pin) return;
            if (node->children.empty()) {
                Disconnect(node);
            } else if (node->children.size() == 1) {
                auto oldParent = node->parent, oldChild = node->children[0];
                Disconnect(node);
                Disconnect(oldChild);
                Connect(oldChild, oldParent);
            }
        });
    }
}

void DelayRefine::Substitute_critical_delay_aware_1(Tree& tree, double eps, double rd, bool useRTree) {
    bgi::rtree<RNode, bgi::rstar<8>, bgi::indexable<RNode>, RNodeComp> rtree;
    if (useRTree) {
        tree.postOrder([&](const shared_ptr<TreeNode>& n) {
            if (n->parent) {
                BBox s;
                bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(n->parent->loc.x, n->parent->loc.y)), s);
                rtree.insert({s, n});
            }
        });
    }
    auto Disconnect = [&](const shared_ptr<TreeNode>& n) {
        if (useRTree) {
            BBox s;
            bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(n->parent->loc.x, n->parent->loc.y)), s);
            rtree.remove({s, n});
        }
        TreeNode::resetParent(n);
    };
    auto Connect = [&](const shared_ptr<TreeNode>& n, const shared_ptr<TreeNode>& parent) {
        TreeNode::setParent(n, parent);
        if (useRTree) {
            BBox s;
            bg::envelope(BSegment(BPoint(n->loc.x, n->loc.y), BPoint(parent->loc.x, parent->loc.y)), s);
            rtree.insert({s, n});
        }
    };

    auto make_node_id = 10*tree.UpdateId();

    auto outchild = [&](const shared_ptr<TreeNode>& node) {
        if (node->children.empty()) {
            cout << "Node " << node->id << " has no children" << endl;
        } else {
            cout << "Node " << node->id << " has children: ";
            for (auto c : node->children) {
                cout << c->id << " ";
            }
            cout << endl;
        }
    };

    // 预备
    salt::ElmoreDelayEval before_delayEval(rd, tree);
    auto before_sumDelay = before_delayEval.sumNorDelay;

    auto for_one_net_maxLB = before_delayEval._maxLb;

    // 最差的delay
    int max_delay_node_id = before_delayEval.max_delay_node_id;
    auto max_delay = before_delayEval.nodeNorDelay[max_delay_node_id];

    int iter_num = 0;

    // comment_info 统计改变topology和计算delay所用的时间
    TimeStat time_stat;

    while (true) {
    iter_num++;
        // Get nearest neighbors
        int num = tree.UpdateId();
        cout << "num: " << num << endl;
        vector<shared_ptr<TreeNode>> nodes = tree.ObtainNodes(),
                                     orderedNodes(nodes.size());  // note: all pins should be covered
        if (nodes.size() != num) {
            cout << "debug\n";
        }
        vector<Point> points(nodes.size());
        for (int i = 0; i < nodes.size(); ++i) {
            // cout << "node " << nodes[i]->id << endl;
            orderedNodes[nodes[i]->id] = nodes[i];
            points[nodes[i]->id] = nodes[i]->loc;  // TODO: move within bracket
        }
        nodes = orderedNodes;
        vector<vector<int>> nearestNeighbors;
        if (!useRTree) {
            MstBuilder mstB;
            mstB.GetAllNearestNeighbors(points, nearestNeighbors);
        } else {
            nearestNeighbors.resize(nodes.size());
            for (auto n : nodes) {
                if (n->parent) {
                    Point c = n->loc;  // center
                    DTYPE radius = n->WireToParent();
                    // diamond is too slow...
                    // BPolygon diamond;
                    // diamond.outer().emplace_back(c.x - 3, c.y);
                    // diamond.outer().emplace_back(c.x, c.y + 3);
                    // diamond.outer().emplace_back(c.x + 3, c.y);
                    // diamond.outer().emplace_back(c.x, c.y - 3);
                    // diamond.outer().emplace_back(c.x - 3, c.y);
                    int tolrance = 1;
                    BBox queryBox{{c.x - tolrance*radius, c.y - tolrance*radius}, {c.x + tolrance*radius, c.y + tolrance*radius}};
                    // BBox queryBox{{c.x - 3, c.y - 3}, {c.x + 3, c.y + 3}};
                    vector<RNode> cands;
                    rtree.query(bgi::intersects(queryBox), back_inserter(cands));  // TODO: change back_inserter
                    for (const auto& cand : cands) {
                        // if (n->id == 2 && cand.second->id == 4) {
                        //     continue;
                        // }
                        nearestNeighbors[n->id].push_back(cand.second->id);
                    }
                }
            }
        }

        // Prune descendants in nearest neighbors
        vector<int> preOrderIdxes(nodes.size(), -1);
        int globalPreOrderIdx = 0;
        function<void(const shared_ptr<TreeNode>&)> removeDescendants = [&](const shared_ptr<TreeNode>& node) {
            preOrderIdxes[node->id] = globalPreOrderIdx++;
            for (auto child : node->children) {
                removeDescendants(child);
            }
            for (auto& neighIdx : nearestNeighbors[node->id]) {
                int neighPreOrderIdx = preOrderIdxes[neighIdx];
                if (neighPreOrderIdx != -1 && neighPreOrderIdx >= preOrderIdxes[node->id]) {
                    neighIdx = -1;  // -1 stands for "descendant"
                }
            }
        };
        removeDescendants(tree.source);

        // Init path lengths and subtree slacks
        vector<DTYPE> pathLengths(nodes.size());
        vector<DTYPE> slacks(nodes.size());
        auto UpdatePathLengths = [&](const shared_ptr<TreeNode>& node) {
            if (node->parent) {
                pathLengths[node->id] = pathLengths[node->parent->id] + node->WireToParent();
            } else {
                pathLengths[node->id] = 0;
            }
        };
        auto UpdateSlacks = [&](const shared_ptr<TreeNode>& node) {
            if (node->children.empty()) {
                slacks[node->id] =
                    Dist(node->loc, tree.source->loc) * (1 + eps) - pathLengths[node->id];  // floor here...
            } else {
                DTYPE minSlack = Dist(node->loc, tree.source->loc) * (1 + eps) - pathLengths[node->id];
                for (auto child : node->children) {
                    minSlack = min(minSlack, slacks[child->id]);
                }
                slacks[node->id] = minSlack;
            }
        };
        tree.preOrder(UpdatePathLengths);
        tree.postOrder(UpdateSlacks);

        // Find legal candidate moves
        // using MoveT = tuple<DTYPE, shared_ptr<TreeNode>, shared_ptr<TreeNode>>;
        using MoveT = tuple<double, shared_ptr<TreeNode>, shared_ptr<TreeNode>, int>;  // 最后一个的int，1表示选neigh，2表示选neighParent，3表示选steinerPt
        int which_node = 0;
        vector<MoveT> candidateMoves;  // <wireLengthDelta, node, newParent>
        auto GetNearestPoint = [](const shared_ptr<TreeNode>& target, const shared_ptr<TreeNode>& neigh) {
            Box box(neigh->loc, neigh->parent->loc);
            box.Legalize();
            return box.GetNearestPointTo(target->loc);
        };

        // vector<double> cap(num, 0);
        // tree.postOrder([&](const shared_ptr<TreeNode>& node) {
        //     if (node->pin && node != tree.source) cap[node->id] = node->pin->cap;
        //     for (auto c : node->children) {
        //         cap[node->id] += cap[c->id];
        //         cap[node->id] += c->WireToParent() * 8e-20;
        //     }
        // });

        for (auto node : nodes) {
            if (node->id != 2) {
                continue;
            }
            std::shared_ptr<salt::TreeNode> tmp_steinerNode = nullptr;
            if (!(node->parent)) {
                continue;
            }

            auto originalParent = node->parent;
            DTYPE bestWireLengthDelta = 0;  // the negative, the better

            double best_delay_delta = 0;  // the negative, the better

            shared_ptr<TreeNode> bestNewParent;
            for (int neighIdx : nearestNeighbors[node->id]) {
                if (node->id == 2 && neighIdx == 41) {
                    cout << "afadsfn";
                }
                // bug   why this?
                if (neighIdx >= nodes.size()) continue;
                // bug   why this?
                if (neighIdx == -1 || !nodes[neighIdx]->parent) continue;
                auto neigh = nodes[neighIdx];
                auto neighParent = neigh->parent;

                // bug   why this?
                if (node == neighParent) continue;
                if (node == neigh) continue;
                // bug   why this?

                // pl不能违规
                DTYPE pathLengthDelta =
                        pathLengths[neighParent->id] + Dist(node->loc, neighParent->loc) - pathLengths[node->id];
                if (pathLengthDelta > slacks[node->id]) continue;

                auto steinerPt = GetNearestPoint(node, neigh);

                bool exct_twice = (steinerPt == neigh->loc) || (steinerPt == neighParent->loc) ? 1 : 0;
                vector<shared_ptr<TreeNode>> tmp_nodes;
                tmp_nodes.emplace_back(neigh);
                tmp_nodes.emplace_back(neighParent);
                for (int i = 0; i < 2; i++) {
                    // comment_info 改变topology所花费的时间
                    utils::timer topo_change_Timer;
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, tmp_nodes[i]);
                    double topo_change_time = topo_change_Timer.elapsed();
                    time_stat.Inc_change_topo(topo_change_time);
                    // comment_info 改变topology所花费的时间

                    // comment_info 计算delay所花费的时间
                    utils::timer delay_Timer;
                    // salt::ElmoreDelayEval delayEval(rd, tree);
                    salt::ElmoreDelayEval delayEval(rd, tree, for_one_net_maxLB);
                    // auto sumDelay = 100;//delayEval.sumNorDelay;
                    // auto sumDelay = delayEval.sumNorDelay;
                    auto after_max_delay = delayEval.nodeNorDelay[max_delay_node_id];
                    double delay_time = delay_Timer.elapsed();
                    time_stat.Inc_calc_delay(delay_time);
                    // comment_info 计算delay所花费的时间

                    if (after_max_delay + 1e-12 < max_delay) {
                        auto delay_delta = after_max_delay - max_delay;
                        if (delay_delta < best_delay_delta) {
                            best_delay_delta = delay_delta;
                            bestNewParent = neigh;
                            which_node = i+1;
                        }
                    }

                    // comment_info 恢复topology所花费的时间
                    utils::timer recover_timer;
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, originalParent);
                    double recover_time = recover_timer.elapsed();
                    time_stat.Inc_recover_topo(recover_time);
                    // comment_info 恢复topology所花费的时间
                }

                tmp_steinerNode = nullptr;
                if (!exct_twice) {
                    // comment_info 改变topology所花费的时间
                    utils::timer topo_change_Timer;
                    tmp_steinerNode = make_shared<TreeNode>(steinerPt);
                    TreeNode::setParent(tmp_steinerNode, neighParent);
                    TreeNode::resetParent(neigh);
                    TreeNode::setParent(neigh, tmp_steinerNode);
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, tmp_steinerNode);
                    tmp_steinerNode->id = nodes.size();
                    double topo_change_time = topo_change_Timer.elapsed();
                    time_stat.Inc_change_topo(topo_change_time);
                    // comment_info 改变topology所花费的时间

                    // comment_info 计算delay所花费的时间
                    utils::timer delay_Timer;
                    // salt::ElmoreDelayEval delayEval(rd, tree);
                    salt::ElmoreDelayEval delayEval(rd, tree, for_one_net_maxLB);
                    auto after_max_delay = delayEval.nodeNorDelay[max_delay_node_id];
                    double delay_time = delay_Timer.elapsed();
                    time_stat.Inc_calc_delay(delay_time);
                    // comment_info 计算delay所花费的时间

                    if (after_max_delay + 1e-12 < max_delay) {
                        auto delay_delta = after_max_delay - max_delay;
                        if (delay_delta < best_delay_delta) {
                            best_delay_delta = delay_delta;
                            bestNewParent = neigh;
                            which_node = 3;
                        }
                    }

                    // comment_info 恢复topology所花费的时间
                    utils::timer recover_timer;
                    TreeNode::resetParent(tmp_steinerNode);
                    TreeNode::resetParent(node);
                    TreeNode::resetParent(neigh);
                    TreeNode::setParent(node, originalParent);
                    TreeNode::setParent(neigh, neighParent);
                    double recover_time = recover_timer.elapsed();
                    time_stat.Inc_recover_topo(recover_time);
                    // comment_info 恢复topology所花费的时间
                }

            }
            if (bestNewParent) {
                assert(which_node != 0);
                candidateMoves.emplace_back(best_delay_delta, node, bestNewParent, which_node);
                // candidateMoves.emplace_back(bestWireLengthDelta, node, bestNewParent);
            }
            tmp_steinerNode = nullptr;
            tmp_steinerNode.reset();
        }
        if (candidateMoves.empty()) {
            break;
        }
        // cout << tree;


        // Try candidate moves in the order of descending wire length savings
        // Note that earlier moves may influence the legality of later one
        sort(candidateMoves.begin(), candidateMoves.end(), [](const MoveT& lhs, const MoveT& rhs){
            return get<0>(lhs) < get<0>(rhs);
        });
        for (const auto& move : candidateMoves) {
            auto node = get<1>(move), neigh = get<2>(move);
            auto which_node = get<3>(move);
            auto neighParent = neigh->parent;
            // check due to earlier moves
            if (TreeNode::isAncestor(node, neighParent)) continue;

            DTYPE pathLengthDelta =
                pathLengths[neighParent->id] + Dist(node->loc, neighParent->loc) - pathLengths[node->id];
            // if (pathLengthDelta > slacks[node->id]) continue;

            auto steinerPt = GetNearestPoint(node, neigh);

            // explain 预估一下timing
            auto originalParent = node->parent;
            std::shared_ptr<salt::TreeNode> tmp_steinerNode = nullptr;
                if (which_node == 1) {
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, neigh);
                    // Connect(node, neigh);
                } else if (which_node == 2) {
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, neighParent);
                    // Connect(node, neighParent);
                } else {
                    tmp_steinerNode = make_shared<TreeNode>(steinerPt);
                    TreeNode::setParent(tmp_steinerNode, neighParent);
                    // Disconnect(neigh);
                    TreeNode::resetParent(neigh);
                    TreeNode::setParent(neigh, tmp_steinerNode);
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, tmp_steinerNode);
                    // for later moves
                    tmp_steinerNode->id = nodes.size();   
                }

                // cout << tree;

                // salt::ElmoreDelayEval delayEval(rd, tree);
                salt::ElmoreDelayEval delayEval(rd, tree, for_one_net_maxLB);
                // auto sumDelay = 100;//delayEval.sumNorDelay;
                // auto sumDelay = delayEval.sumNorDelay;
                auto after_max_delay = delayEval.nodeNorDelay[max_delay_node_id];


                if (which_node == 1) {
                    // TreeNode::setParent(node, neigh);
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, originalParent);
                } else if (which_node == 2) {
                    // TreeNode::setParent(node, neighParent);
                    TreeNode::resetParent(node);
                    TreeNode::setParent(node, originalParent);
                } else {
                    TreeNode::resetParent(tmp_steinerNode);
                    TreeNode::resetParent(node);
                    TreeNode::resetParent(neigh);
                    TreeNode::setParent(node, originalParent);
                    TreeNode::setParent(neigh, neighParent);
                }

                tmp_steinerNode.reset();

                if (after_max_delay + 1e-12 < max_delay) {
                    max_delay = after_max_delay;
                } else {
                    continue;
                }
            // explain 预估一下timing

            // break
            Disconnect(node);
            // reroot
            if (which_node == 1) {
                Connect(node, neigh);
            } else if (which_node == 2) {
                Connect(node, neighParent);
            } else {
                auto steinerNode = make_shared<TreeNode>(steinerPt);
                Connect(steinerNode, neighParent);
                Disconnect(neigh);
                Connect(neigh, steinerNode);
                Connect(node, steinerNode);
                // for later moves
                steinerNode->id = nodes.size();
                // steinerNode->id = make_node_id++;
                nodes.push_back(steinerNode);
                pathLengths.push_back(pathLengths[neighParent->id] + steinerNode->WireToParent());
                slacks.push_back(Dist(steinerNode->loc, tree.source->loc) * (1 + eps) - pathLengths.back());
            }
            // update slack for later moves: first subtree, then path to source
            TreeNode::preOrder(neighParent, UpdatePathLengths);
            TreeNode::postOrder(neighParent, UpdateSlacks);

            // update timing info
            // salt::ElmoreDelayEval delayEval(rd, tree);
            // auto updated_timing = delayEval.sumDelay;
            tree.UpdateId();
            // update timing info


            auto tmp = neighParent;
            while (tmp->parent) {
                slacks[tmp->parent->id] = min(slacks[tmp->parent->id], slacks[tmp->id]);
                tmp = tmp->parent;
            }
        }

        // Finalize
        tree.RemoveTopoRedundantSteiner();
        // cout << tree;
        tree.postOrderCopy([&](const shared_ptr<TreeNode>& node) {
            // degree may change after post-order traversal of its children
            if (node->pin) return;
            if (node->children.empty()) {
                Disconnect(node);
            } else if (node->children.size() == 1) {
                auto oldParent = node->parent, oldChild = node->children[0];
                Disconnect(node);
                Disconnect(oldChild);
                Connect(oldChild, oldParent);
            }
        });
    }

    // comment_info 输出运行时间情况
    std::cout << "\033[1;33m";
    cout << " Change topo " << time_stat.change_topo_cnt << " times, total run time " << time_stat.change_topo_time << endl;
    cout << "Recover topo " << time_stat.recover_topo_cnt << " times, total run time " << time_stat.recover_topo_time << endl;
    cout << "  Calc delay " << time_stat.calc_delay_cnt << " times, total run time " << time_stat.calc_delay_time << endl;
    std::cout << "\033[0m";
}

}  // namespace salt