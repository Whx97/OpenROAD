#pragma once

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <numeric>
#include <vector>

// #include "salt/base/flute.h"
// #include "salt/base/rsa.h"
// #include "salt/refine/refine.h"
// #include "salt/base/tree.h"
// #include "salt/utils/utils.h"

// #include "flute.h"
#include "refine/delay_refine.h"

#include <boost/functional/hash.hpp>
#include <unordered_map>

#include "base/eval.h"
// #include "salt/base/flute/flute.h"
#include <limits>
#include "utils/utils.h"
#include "../flute3/flute.h"

// #include "Utility.h"

// #include "api/TimingEngine.hh"
// #include "RoutingTree.h"
// using PinLoc = std::tuple<ista::DesignObject*, ito::Point>;
namespace salt {
// using namespace Flute;
#define MAXD 9000  // max. degree that can be handled
class SelfBuilder {
public:
    SelfBuilder(int pin_num) : _pin_num(pin_num), _length_to_source(2 * pin_num), _length_Mah_source(2 * pin_num) {}
    double run_flute(const salt::Net& net, salt::Tree& saltTree);
    // void Run(const Net& net, Tree& tree, double eps, ista::TimingEngine *_timing_engine, ista::Net *sta_net, const vector<PinLoc> pin2loc);
    void Run(const Net& net, Tree& tree, double eps);

    vector<DTYPE> shortestDists;
    vector<DTYPE> curDists;
    vector<shared_ptr<TreeNode>> slNodes; // nodes of the shallow-light tree
    shared_ptr<TreeNode> slSrc;  // source node of the shallow-light tree

    void Init(Tree& minTree, shared_ptr<Pin> srcP) {
        // minTree.UpdateId();
        auto mtNodes = minTree.ObtainNodes();
        slNodes.resize(mtNodes.size());
        shortestDists.resize(mtNodes.size());
        curDists.resize(mtNodes.size());
        for (auto mtN : mtNodes) {
            // slNodes[mtN->id] = make_shared<TreeNode>(mtN->loc, mtN->pin, mtN->id);
            slNodes[mtN->id] = mtN;
            shortestDists[mtN->id] = Dist(mtN->loc, srcP->loc);
            curDists[mtN->id] = (std::numeric_limits<DTYPE>::max)();
        }
        curDists[srcP->id] = 0;
        slSrc = slNodes[srcP->id];
    }

    bool Relax(const shared_ptr<TreeNode>& u, const shared_ptr<TreeNode>& v);  // from u to v
    void DFS(const shared_ptr<TreeNode>& mstNode, const shared_ptr<TreeNode>& slNode, double eps);


private:
    int _pin_num;
    DTYPE _best_WL;
    DTYPE _current_WL;
    shared_ptr<TreeNode> _source;
    // 在树上，到source的长度
    vector<int> _length_to_source;
    // 到source的曼哈顿距离
    vector<int> _length_Mah_source;

    double _eps;

    int _make_steiner_id;

    void update_length(shared_ptr<TreeNode> node) {
        auto father_id = node->id;
        auto childs = node->children;
        for (auto child : childs) {
            auto child_id = child->id;
            _length_to_source[child_id] = _length_to_source[father_id] + child->WireToParent();
            update_length(child);
        }
    }

    int edge_replace_cost(shared_ptr<salt::TreeNode> add_1,
                          shared_ptr<salt::TreeNode> add_2,
                          shared_ptr<salt::TreeNode> rem_1,
                          shared_ptr<salt::TreeNode> rem_2);

    shared_ptr<salt::TreeNode> find_worst_stretch_node(shared_ptr<salt::TreeNode> source) {
        DTYPE worst = INT_MIN;
        shared_ptr<salt::TreeNode> worst_node = nullptr;

        function<void(const shared_ptr<TreeNode>&, DTYPE)> traverse = [&](const shared_ptr<TreeNode>& node,
                                                                          DTYPE curDist) {
            _length_to_source[node->id] = curDist;
            auto node_to_source_manh_dis = _length_Mah_source[node->id];
            for (auto c : node->children) traverse(c, curDist + c->WireToParent());
            if (node->id < _pin_num) {
                if ((curDist - node_to_source_manh_dis) > worst) {
                    worst = curDist - node_to_source_manh_dis;
                    worst_node = node;
                }
            }
        };
        traverse(source, 0);
        return worst_node;
    }
    vector<pair<shared_ptr<TreeNode>, int>> find_all_stretch_node(shared_ptr<salt::TreeNode> source, Tree& tree) {
        vector<pair<shared_ptr<TreeNode>, int>> stretch_node;

        function<void(const shared_ptr<TreeNode>&, DTYPE)> traverse = [&](const shared_ptr<TreeNode>& node,
                                                                          DTYPE curDist) {
            // log() << "node id: " << node->id << endl;
            _length_to_source[node->id] = curDist;
            auto node_to_source_manh_dis = _length_Mah_source[node->id];
            for (auto c : node->children) traverse(c, curDist + c->WireToParent());
            if (node->id < _pin_num) {
                // if (curDist > 1.115* node_to_source_manh_dis) {  // 26
                if (curDist > _eps * node_to_source_manh_dis) {
                    // if (curDist > node_to_source_manh_dis) {
                    stretch_node.push_back(make_pair(node, curDist - node_to_source_manh_dis));
                    // stretch_node.push_back(make_pair(node, node_to_source_manh_dis));
                    // log() << "==============stretch node: " << node->id << endl;
                }
            }
        };
        traverse(source, 0);
        sort(stretch_node.begin(),
             stretch_node.end(),
             [](pair<shared_ptr<TreeNode>, int> p1, pair<shared_ptr<TreeNode>, int> p2) {
                 return p1.second > p2.second;
                 // return p1.second < p2.second;
             });
        // std::reverse(stretch_node.begin(), stretch_node.end());
        


        // tree.PreOrder([&](shared_ptr<TreeNode> node) {
        //     // cout << node->id << endl;
        //     auto node_to_source_manh_dis = _length_Mah_source[node->id];
        //     if (node->id < _pin_num) {
        //         // if (curDist > 1.115* node_to_source_manh_dis) {  // 26
        //         if (_length_to_source[node->id] > _eps * node_to_source_manh_dis) {
        //             // if (curDist > node_to_source_manh_dis) {
        //             stretch_node.push_back(make_pair(node, _length_to_source[node->id] - node_to_source_manh_dis));
        //             // stretch_node.push_back(make_pair(node, node_to_source_manh_dis));
        //             // log() << "==============stretch node: " << node->id << endl;
        //         }
        //     }
        // });
        return stretch_node;
    }

    bool fix_stretch(Tree& tree, shared_ptr<salt::TreeNode> node, double eps, DTYPE best_WL);
    bool fix_wire_length(Tree& tree, shared_ptr<salt::TreeNode> node, double eps, DTYPE best_WL);

    bool dfs(shared_ptr<TreeNode> current,
             const shared_ptr<TreeNode> destination,
             std::vector<shared_ptr<TreeNode>>& path) {
        path.push_back(current);

        if (current == destination) {
            return true;
        }

        for (auto child : current->children) {
            if (dfs(child, destination, path)) {
                return true;
            }
        }

        path.pop_back();
        return false;
    }
    vector<shared_ptr<TreeNode>> find_path(shared_ptr<TreeNode> s_node, shared_ptr<TreeNode> e_node);
    vector<shared_ptr<TreeNode>> find_path1(shared_ptr<TreeNode> s_node, shared_ptr<TreeNode> e_node);

    void Flip_order(shared_ptr<TreeNode> p_node, shared_ptr<TreeNode> c_node, shared_ptr<TreeNode> c_node_father);

    bool check_WL(shared_ptr<salt::TreeNode> node, shared_ptr<salt::TreeNode> next_node, double eps) {
        auto path = find_path1(node, next_node);
        for (int edge_id = 0; edge_id < path.size() - 1; edge_id++) {  // 找到需要移除哪条边
            auto node_1 = path[edge_id];
            auto node_2 = node_1->parent;

            // 把（node_1，node_2）这条边移除的cost
            DTYPE add_edge_length = utils::Dist(node->loc, next_node->loc);
            DTYPE remove_edge_length = utils::Dist(node_1->loc, node_2->loc);
            DTYPE changed_WL = _best_WL + (add_edge_length - remove_edge_length);
            // cout << "remove edge " << node_1->id << ", " << node_2->id << endl;
            if (changed_WL < eps * _best_WL) {
                return true;
            }
        }
        return true;
    }

    /**
     * @brief 父节点位于 子节点和根节点的中间
     *
     * @param parent
     * @param node
     * @param source
     * @return true
     * @return false
     */
    bool isBetweenPoints(const shared_ptr<TreeNode>& parent,
                         const shared_ptr<TreeNode>& node,
                         const shared_ptr<TreeNode>& source) {
        auto source_x = source->loc.x;
        auto source_y = source->loc.y;
        auto parent_x = parent->loc.x;
        auto parent_y = parent->loc.y;
        auto node_x = node->loc.x;
        auto node_y = node->loc.y;
        if (parent_x <= max(node_x, source_x) && parent_x >= min(node_x, source_x)) {
            if (parent_y <= max(node_y, source_y) && parent_y >= min(node_y, source_y)) {
                return true;
            }
        }
        return false;
    }

    // Find the ancestor closest to the source
    // 找附近的点，并且只要保证PL满足某个范围，而且WL不能增加太多
    shared_ptr<salt::TreeNode> find_ancestor_closest_source(shared_ptr<salt::TreeNode> node, Tree& tree) {
        // shared_ptr<salt::TreeNode> next_node = node->parent;
        // while (!isBetweenPoints(next_node, node, _source)) {
        //     next_node = next_node->parent;
        // }
        // if (node->id == 16) {
        //     cout << endl;
        // }

        shared_ptr<salt::TreeNode> next_node = nullptr;
        shared_ptr<salt::TreeNode> next_node5 = nullptr;
        int extra_wl = INT_MAX;
        tree.postOrderCopy([&](shared_ptr<TreeNode> nd) {
            // 两个点不能有连接
            if (nd->id != node->id && node->parent != nd && nd->parent != node) {
                // if (nd->id ==1) {
                //     cout << endl;
                // }
                // if (nd->id ==12) {
                //     cout << endl;
                // }

                // DTYPE node_to_source_dis = node->WireToSource();
                auto dist = utils::Dist(nd->loc, node->loc);
                auto pl = dist + _length_to_source[nd->id];
                // auto pl = dist + _length_Mah_source[nd->id];
                // 1.03 for 17
                auto compare_length = _length_to_source[nd->id] > _length_Mah_source[node->id]
                                          ? _eps * _length_Mah_source[node->id]
                                          : _length_Mah_source[node->id];
                if (pl <= _eps * _length_Mah_source[node->id] && dist < extra_wl) {
                    // if (pl < 1.05*_length_Mah_source[node->id] && dist < extra_wl) {
                    if (node->parent->id != nd->id) {
                        next_node = nd;
                        extra_wl = dist;
                    }
                }
            }
        });

        // shared_ptr<salt::TreeNode> next_node = nullptr;
        // shared_ptr<salt::TreeNode> next_node5 = nullptr;
        // int extra_wl = INT_MAX;
        // int min_pl = INT_MAX;
        // tree.PostOrderCopy([&](shared_ptr<TreeNode> nd) {
        //     if (nd->id != node->id) {
        //         auto dist = utils::Dist(nd->loc, node->loc);
        //         auto pl = dist + 1.3*_length_Mah_source[nd->id];
        //         // 1.03 for 17
        //         if (pl < min_pl && dist < extra_wl) {
        //             next_node = nd;
        //             extra_wl = dist;
        //             min_pl = pl;
        //         }
        //     }
        // });
        // cout << "node: " << node->id << " best neighbor " << next_node->id << endl;

        return next_node;
    }

    // Find the ancestor closest to the source
    // 找附近的点，并且只要保证PL满足某个范围，而且WL不能增加太多
    shared_ptr<salt::TreeNode> find_father_closest_source_across_path(shared_ptr<salt::TreeNode> node, Tree& tree) {
        // auto path = find_path1(node, _source);
        // int path_length = path.size();
        // // 如果边不是垂直或平行边，那么需要判断拐角点，拐角点有两个，选择最靠近node的那个
        // for (int i = 1; i < path_length - 1; i++) {
        //     auto node_1 = path[i];
        //     auto node_2 = path[i + 1];
        //     if (node_1->loc.x != node_2->loc.x && node_1->loc.y != node_2->loc.y) {
        //         auto node_3 = path[i + 2];
        //         auto node_4 = path[i + 3];
        //         if (node_3->loc.x == node_4->loc.x || node_3->loc.y == node_4->loc.y) {
        //             return node_1;
        //         }
        //     }
        // }

        // shared_ptr<salt::TreeNode> prev_node = node;
        // while (node->parent != nullptr) {
        //     auto except_node = node->parent;
        //     auto dist = utils::Dist(except_node->loc, node->loc);
        //     auto pl = dist + _length_to_source[except_node->id];

        //     // 连接到某个祖先后满足pl的要求，那么就要判断是否需要额外增加一个点
        //     if (pl < _eps * _length_Mah_source[node->id]) { 
                
        //     }
        //     prev_node = except_node;
        // }

        



        shared_ptr<salt::TreeNode> next_node = nullptr;
        shared_ptr<salt::TreeNode> next_node5 = nullptr;
        int extra_wl = INT_MAX;
        // tree.PostOrderCopy([&](shared_ptr<TreeNode> nd) {
        auto parent = node->parent;
        while (parent != nullptr) {
            auto nd = parent;
            // 两个点不能有连接
            if (nd->id != node->id) {
                // if (nd->id ==17) {
                //     cout << endl;
                // }
                // if (nd->id ==4) {
                //     cout << endl;
                // }

                // DTYPE node_to_source_dis = node->WireToSource();
                auto dist = utils::Dist(nd->loc, node->loc);
                auto pl = dist + _length_to_source[nd->id];
                // auto pl = dist + _length_Mah_source[nd->id];
                // 1.03 for 17
                auto compare_length = _length_to_source[nd->id] > _length_Mah_source[node->id]
                                          ? _eps * _length_Mah_source[node->id]
                                          : _length_Mah_source[node->id];
                if (pl < _eps * _length_Mah_source[node->id] && dist < extra_wl) {
                    // if (pl < 1.05*_length_Mah_source[node->id] && dist < extra_wl) {
                    // if (node->parent->id != nd->id) {
                        next_node = nd;
                        extra_wl = dist;
                    // }
                }
            }
            parent = nd->parent;
        }
        // });
        return next_node;
    }

    void post_processing(Tree& tree);
    void post_processing1(Tree& tree);

    bool stretch_under_constrain() {
        bool under_constrain = true;
        for (int i = 0; i < _pin_num; i++) {
            auto stretch = (double)_length_to_source[i] / _length_Mah_source[i];
            if (stretch > _eps) {
                under_constrain = false;
                return false;
            }
        }
        return under_constrain;
    }

    double calc_avg_stretch() {
        double avg_stretch = 0.0;
        for (int i = 0; i < _pin_num; i++) {
            auto stretch = (double)_length_to_source[i] / _length_Mah_source[i];
            avg_stretch += stretch;
        }
        auto numSink = _pin_num - 1;
        avg_stretch /= numSink;
        return avg_stretch;
    }
};
}  // namespace salt