#include "self.h"
#include "refine/refine.h"


#include <cmath>

namespace salt {

bool only_flute = 0;
bool use_salt_calss = 1;

#define COUT_RED_START std::cout << "\033[1;31m";
#define COUT_GREEN_START std::cout << "\033[1;32m";
#define COUT_YELLOW_START std::cout << "\033[1;33m";
#define COUT_BLUE_START std::cout << "\033[1;34m";
#define COUT_PURPLE_START std::cout << "\033[1;35m";
#define COUT_CYAN_START std::cout << "\033[1;36m";

#define COUT_COLOR_END std::cout << "\033[0m";

vector<shared_ptr<TreeNode>> get_children(shared_ptr<TreeNode> node) { return node->children; }

// vector<int> Break_points_1;
vector<pair<shared_ptr<TreeNode>, int>> Break_points_1;
bool SelfBuilder::Relax(const shared_ptr<TreeNode>& u, const shared_ptr<TreeNode>& v) {
    DTYPE newDist = curDists[u->id] + Dist(u->loc, v->loc);
    if (curDists[v->id] > newDist) {
        curDists[v->id] = newDist;
        // v->parent = u;
        return true;
    } else if (curDists[v->id] == newDist && Dist(u->loc, v->loc) < v->WireToParentChecked()) {
        // v->parent = u;
        return true;
    } else
        return false;
}

void SelfBuilder::DFS(const shared_ptr<TreeNode>& smtNode, const shared_ptr<TreeNode>& slNode, double eps) {
    if (smtNode->pin && curDists[slNode->id] > (1 + eps) * shortestDists[slNode->id]) {
        // slNode->parent = slSrc;
        curDists[slNode->id] = shortestDists[slNode->id];
        // Break_points_1.push_back(make_pair(slNode, curDists[slNode->id]-shortestDists[slNode->id]));
        Break_points_1.push_back(make_pair(slNode, _length_to_source[slNode->id] - _length_Mah_source[slNode->id]));
    }
    for (auto c : smtNode->children) {
        Relax(slNode, slNodes[c->id]);
        DFS(c, slNodes[c->id], eps);
        Relax(slNodes[c->id], slNode);
    }
}

double SelfBuilder::run_flute(const salt::Net& net, salt::Tree& saltTree) {
    // load LUT
    static bool once = false;
    if (!once) {
        Flute::readLUT();
        once = true;
    }

    // Obtain Flute tree
    Flute::Tree fluteTree;
    fluteTree.branch = nullptr;
    int d = net.pins.size();
    assert(d <= MAXD);
    int x[MAXD], y[MAXD];
    for (size_t i = 0; i < d; ++i) {
        x[i] = net.pins[i]->loc.x;
        y[i] = net.pins[i]->loc.y;
    }
    if (fluteTree.branch) free(fluteTree.branch);  // is it complete for mem leak?
    fluteTree = Flute::flute(d, x, y, FLUTE_ACCURACY);

    // Build adjacency list
    unordered_map<pair<DTYPE, DTYPE>, shared_ptr<salt::TreeNode>, boost::hash<pair<DTYPE, DTYPE>>> key2node;
    for (auto p : net.pins) {
        key2node[{p->loc.x, p->loc.y}] = make_shared<salt::TreeNode>(p);
    }
    auto& t = fluteTree;

    auto FindOrCreate = [&](DTYPE x, DTYPE y) {
        auto it = key2node.find({x, y});
        if (it == key2node.end()) {
            shared_ptr<salt::TreeNode> node = make_shared<salt::TreeNode>(x, y);
            key2node[{x, y}] = node;
            return node;
        } else
            return it->second;
    };

    for (int i = 0; i < 2 * t.deg - 2; i++) {
        int j = t.branch[i].n;
        if (t.branch[i].x == t.branch[j].x && t.branch[i].y == t.branch[j].y) continue;
        // any more duplicate?
        shared_ptr<salt::TreeNode> n1 = FindOrCreate(t.branch[i].x, t.branch[i].y);
        shared_ptr<salt::TreeNode> n2 = FindOrCreate(t.branch[j].x, t.branch[j].y);
        // printlog(LOG_INFO, "%d - %d\n", n1->pin?n1->pin->id:-1, n2->pin?n2->pin->id:-1);
        n1->children.push_back(n2);
        n2->children.push_back(n1);
    }

    // Reverse parent-child orders
    saltTree.source = key2node[{net.source()->loc.x, net.source()->loc.y}];
    saltTree.SetParentFromUndirectedAdjList();
    saltTree.net = &net;

    auto length = fluteTree.length;

    free(fluteTree.branch);
    return length;
}

bool comparePairs(const shared_ptr<TreeNode>& p1, const shared_ptr<TreeNode>& p2) {
    if (p1.get()->loc.x < p2.get()->loc.x) {
        return true;
    } else if (p1.get()->loc.x == p2.get()->loc.x) {
        return p1.get()->loc.y < p2.get()->loc.y;
    }
}

// void SelfBuilder::Run(const Net& net, Tree& tree, double eps, ista::TimingEngine *_timing_engine, ista::Net *sta_net, const vector<PinLoc> pin2loc) {
void SelfBuilder::Run(const Net& net, Tree& tree, double eps) {
    // // SMT
    // Tree smt;
    // FluteBuilder fluteB;
    // fluteB.Run(net, tree);
    _eps = eps;

    auto best_WL = run_flute(net, tree);
    _best_WL = best_WL;
    if (only_flute) {
        tree.UpdateId();
        return;
    }

    int pin_num = net.pins.size();

    int node_num = tree.UpdateId();
    _make_steiner_id = node_num;
    // int node_num = 0;
    // tree.PreOrder([&](const shared_ptr<TreeNode>& node) {
    //     node_num++;
    // });

    // 对所有节点的坐标进行排序
    // vector<shared_ptr<TreeNode>> sorted_node;
    // tree.PostOrderCopy([&](shared_ptr<TreeNode> node) {
    //     // cout << node.get()->id << " " << node.get()->loc.x << ", " << node.get()->loc.y << endl;
    //     auto n = node;
    //     sorted_node.push_back(n);
    // });

    // sort(sorted_node.begin(), sorted_node.end(), comparePairs);
    // for (auto node : sorted_node) {
    //     cout << node.get()->id << " " << node.get()->loc.x << ", " << node.get()->loc.y << endl;
    // }

    _source = tree.source;

    // cout << tree;

    function<void(const shared_ptr<TreeNode>&, DTYPE)> traverse = [&](const shared_ptr<TreeNode>& node,
                                                                                    DTYPE curDist) {
                        _length_to_source[node->id] = curDist;
                        auto node_to_source_manh_dis = utils::Dist(_source->loc, node->loc);
                        _length_Mah_source[node->id] = node_to_source_manh_dis;
                        for (auto c : node->children) traverse(c, curDist + c->WireToParent());
                    };
                traverse(tree.source, 0);

    Break_points_1.clear();
    Init(tree, net.source());
    DFS(tree.source, slSrc, eps-1);
    // for (auto i : Break_points_1) {
    //     COUT_BLUE_START;
    //     cout << i.first->id << "  ";
    // }
    // cout << endl;
    // COUT_COLOR_END;

    // cout << tree;

    vector<pair<DTYPE, DTYPE>> import_edge;
    // 记录path
    vector<pair<DTYPE, DTYPE>> path_loc;

    int best_PL = INT_MAX;

    int max_iter = 10;
    int iter = 0;
    auto node = find_worst_stretch_node(_source);
    _current_WL = _best_WL;
    // // TODO: 如果连续两次都是同一个点，那么第二次需要选次节点
    // while (node && iter < max_iter) {
    //     cout << "find_worst_stretch_node " << node->id << endl;
    //     bool improve = fix_stretch(tree, node, eps, _current_WL);
    //     if (improve) {
    //         Refine::cancelIntersect(tree);
    //         Refine::flip(tree);
    //         Refine::uShift(tree);
    //         node_num = tree.UpdateId();
    //         node = find_worst_stretch_node(source);
    //         salt::WireLengthEval eval(tree);
    //         _current_WL = eval.wireLength;
    //     }
    //     iter++;
    // }

    // 为了防止从最差的开始优化，但是优化它会导致WL超了，就会一直重复优化这个点
    // 所以需要改成：一直没有优化的时候，不断遍历到下一个点
    max_iter = 20;
    // 最大迭代次数设置为pin的个数
    max_iter = 2*node_num;
    iter = 0;
    vector<std::pair<std::shared_ptr<salt::TreeNode>, int>> all_stretch_node;
    if (!use_salt_calss) {
        all_stretch_node = find_all_stretch_node(_source, tree);
    } else {
        // 换成salt中的找违例点
        all_stretch_node = Break_points_1;
        sort(all_stretch_node.begin(),
                 all_stretch_node.end(),
                 [](pair<shared_ptr<TreeNode>, int> p1, pair<shared_ptr<TreeNode>, int> p2) {
                     return p1.second > p2.second;
                     // return p1.second < p2.second;
                 });
    }


    int stretch_node_num = all_stretch_node.size();
    int idx = 0;
    if (!all_stretch_node.empty()) {
        while (all_stretch_node[idx].first && iter < max_iter) {
            // COUT_PURPLE_START;
            // cout << "all_stretch_node: ";
            // for (auto i : all_stretch_node) {
            //     cout << i.first->id << "  ";
            // }
            // cout << endl;
            // COUT_COLOR_END;
            // COUT_RED_START;
            // cout << "fix_stretch_node " << all_stretch_node[idx].first->id << endl;
            // COUT_COLOR_END;
            bool improve = fix_stretch(tree, all_stretch_node[idx].first, eps, _current_WL);
            // bool improve = fix_stretch(tree, all_stretch_node[idx].first, eps, _best_WL);
            if (improve) {
                // cout << tree;

                // tree.Write("SELF");
                // Refine::cancelIntersect(tree);
                // Refine::flip(tree);
                // Refine::uShift(tree);
                // node_num = tree.UpdateId();

                // tree.PreOrder([&](const shared_ptr<TreeNode>& node) {
                //     auto node_to_source_manh_dis = utils::Dist(_source->loc, node->loc);
                //     _length_Mah_source[node->id] = node_to_source_manh_dis;
                // });
                function<void(const shared_ptr<TreeNode>&, DTYPE)> traverse = [&](const shared_ptr<TreeNode>& node,
                                                                                    DTYPE curDist) {
                        _length_to_source[node->id] = curDist;
                        auto node_to_source_manh_dis = utils::Dist(_source->loc, node->loc);
                        _length_Mah_source[node->id] = node_to_source_manh_dis;
                        for (auto c : node->children) traverse(c, curDist + c->WireToParent());
                    };
                traverse(tree.source, 0);

                if (!use_salt_calss) {
                    all_stretch_node = find_all_stretch_node(_source, tree);
                } else {
                    Break_points_1.clear();
                    Init(tree, net.source());
                    DFS(tree.source, slSrc, eps-1);
                    // for (auto i : Break_points_1) {
                    //     COUT_BLUE_START;
                    //     cout << i.first->id << "  ";
                    // }
                    // cout << endl;
                    // COUT_COLOR_END;
                    all_stretch_node = Break_points_1;
                }

                idx = 0;
                stretch_node_num = all_stretch_node.size();
                iter++;

                // tree.Write("SELF");
                salt::WireLengthEval eval(tree);
                _current_WL = eval.wireLength;
                if (all_stretch_node.empty()) {
                    // cout << "All node stretch are 1\n";
                    break;
                }

            } else {
                idx++;
                if (idx == stretch_node_num) {
                    cout << "Can't opti PL under max WL constraint\n";
                    break;
                }
            }

            // if (calc_avg_stretch() < 1.01) {
            //     break;
            // }
        }
    }

    if (iter == max_iter) {
        COUT_GREEN_START;
        cout << "\t\t\t\t\t\tMax iteration achieved.\n";
        COUT_COLOR_END;
    }


    // tree.PreOrder([&](const shared_ptr<TreeNode>& node) {
    //             auto node_to_source_manh_dis = utils::Dist(_source->loc, node->loc);
    //             _length_Mah_source[node->id] = node_to_source_manh_dis;
    //         });

    // _make_steiner_id = tree.UpdateId();

    // post_processing(tree);

    // Refine::cancelIntersect(tree);
    // Refine::flip(tree);
    // Refine::uShift(tree);
    // _make_steiner_id = tree.UpdateId();
    // post_processing(tree);
    // _make_steiner_id = tree.UpdateId();
    // tree.Write("SELF");
    // Refine::substitute(tree, eps-1, true);

    // Refine::Substitute_critical_delay_aware(tree, eps-1, 4550.758, true);
    // DelayRefine::Substitute_critical_delay_aware_1(tree, eps-1, 4550.758, true);
    // DelayRefine::Substitute_critical_delay_aware_1(tree, eps-1, 5400.47, true);
    // Refine::Substitute_critical_delay_aware_1(tree, eps-1,  90000.758, true);
    // Refine::Substitute_critical_delay_aware_1(tree, eps-1, 45000.758, true);
    // DelayRefine::Substitute_delay_aware(tree, eps-1, 4500.758, true);
    // DelayRefine::Substitute_delay_aware(tree, eps-1, 65000.758, true);
    // DelayRefine::Substitute_delay_aware_1(tree, eps-1, 4500.758, _timing_engine, sta_net, pin2loc);
    // cout << "before========================\n";
    // cout << tree;
    // DelayRefine::Substitute_delay_aware_1(tree, eps, 180000000.545, _timing_engine, sta_net, pin2loc);
    // DelayRefine::Substitute_delay_aware_1(tree, eps-1, 0.0001, _timing_engine, sta_net, pin2loc);//gcd 可以，pl、wl过程不要优化，critical效果要好
    // DelayRefine::Substitute_delay_aware_1(tree, eps-1, 2700.47, _timing_engine, sta_net, pin2loc);
    // DelayRefine::Substitute_delay_aware_1(tree, eps-1, 45000.758, _timing_engine, sta_net, pin2loc);
    // Refine::Substitute_delay_aware_1(tree, eps-1, 45000000.758, true);
    // Refine::Substitute_delay_aware_1(tree, eps-1, 200.47, true);
    // Refine::Substitute_delay_aware_1(tree, eps-1, 4500.758, true);//gcd
    // Refine::Substitute_delay_aware_1(tree, eps-1, 80000.758, true);//Aes|aes_core|apu
    // Refine::Substitute_delay_aware_1(tree, eps-1, 65000.758, true);//blabla|salsa20  
    // Refine::Substitute_delay_aware_1(tree, eps-1, 45000.758, true);//BM64
    // Refine::Substitute_delay_aware_1(tree, eps-1, 20000.758, true);//caravel_upw buyouhua pl
    Refine::Substitute_delay_aware_1(tree, eps-1, 20000.758, true);//ppu|s44
    // Refine::Substitute_delay_aware_1(tree, eps-1, 29.758, true);//
    // Refine::Substitute_delay_aware_1(tree, eps-1, 2700.47, true);//28
    tree.UpdateId();
    // // tree.Write("SELF");
    // post_processing(tree);

    // // test
    // shared_ptr<TreeNode> p_node;
    // shared_ptr<TreeNode> c_node;
    // shared_ptr<TreeNode> c_node_father;
    // tree.PostOrderCopy([&](shared_ptr<TreeNode> node) {
    //     cout << node.get()->id << " " << node.get()->loc.x << ", " << node.get()->loc.y << endl;
    //     auto n = node;
    //     if (n->id == 7) {
    //         p_node = n;
    //     }
    //     if (n->id == 1) {
    //         c_node = n;
    //     }
    //     if (n->id == 0) {
    //         c_node_father = n;
    //     }
    // });
    // auto pp = find_path1(p_node, c_node);
    // for (auto p : pp) {
    //     cout << p->id << ", ";
    // }
    // cout << endl;
    // Flip_order(p_node, c_node, c_node_father);


    // // Refine SMT
    // if (refineLevel >= 1) {
    //     Refine::flip(smt);
    //     Refine::uShift(smt);
    // }

    // // Init
    // Init(smt, net.source());

    // // DFS
    // DFS(smt.source, slSrc, eps);
    // Finalize(net, tree);
    // tree.RemoveTopoRedundantSteiner();

    // // Connect breakpoints to source by RSA
    // salt::RsaBuilder rsaB;
    // rsaB.ReplaceRootChildren(tree);

    // // Refine SALT
    // if (refineLevel >= 1) {
    //     Refine::cancelIntersect(tree);
    //     Refine::flip(tree);
    //     Refine::uShift(tree);
    //     if (refineLevel >= 2) {
    //         Refine::Substitute(tree, eps, refineLevel == 3);
    //     }
    // }
}

bool SelfBuilder::fix_stretch(Tree& tree, shared_ptr<salt::TreeNode> node, double eps, DTYPE current_WL) {
    eps = 4;
    bool improve = false;

    // // 对所有节点的坐标进行排序
    // vector<shared_ptr<TreeNode>> sorted_node = tree.ObtainNodes();
    // sort(sorted_node.begin(), sorted_node.end(), comparePairs);
    // cout << "asdofajsd\n";
    // for (auto node : sorted_node) {
    // cout << node.get()->id << " " << node.get()->loc.x << ", " << node.get()->loc.y << endl;
    // }

    // 记录没优化前的长度信息
    vector<int> length_to_source;
    vector<int> length_Mah_source;
    length_to_source = _length_to_source;
    length_Mah_source = _length_Mah_source;

    int best_PL = INT_MAX;

    auto parent = node->parent;  // 父节点
    if (parent) {
        auto parent_x = parent->loc.x;
        auto parent_y = parent->loc.y;
        auto node_x = node->loc.x;
        auto node_y = node->loc.y;
        // if (isBetweenPoints(parent, node, _source)) {  // 父节点位于 子节点和根节点的中间
        //     // import_edge.push_back(
        //     //     make_pair(min(node->id, parent->id), max(node->id, parent->id)));  // 标记边为重要的
        // } else {  // 父节点不在 子节点和根节点的中间，需要找到另外一个点，并把当前子节点连到它，然后还要删除一条边
            // auto find_node = find(sorted_node.begin(),
            //                       sorted_node.end(),
            //                       node);  // 在排序后的节点中找到当前节点，然后在这个附近找下一个点

            std::shared_ptr<salt::TreeNode> next_node = nullptr;
            // if (node_x <= _source->loc.x) {  // 当前节点位于根节点的左侧
            //     next_node = *(++find_node);
            //     while (next_node) {
            //         if (next_node->loc.y <= max(node_y, _source->loc.y) &&
            //             next_node->loc.y >= min(node_y, _source->loc.y)) {
            //             // next_node 位于root和node中间，但是线长太长的话也需要继续往下找
            //             bool check = check_WL(node, next_node, eps);
            //             if (check) {
            //                 break;
            //             }

            //             // break;
            //             next_node = *(++find_node);
            //         } else {
            //             next_node = *(++find_node);
            //         }
            //     }
            // } else {  // 当前节点位于根节点的右侧
            //     next_node = *(--find_node);
            //     while (next_node) {
            //         if (next_node->loc.y <= max(node_y, _source->loc.y) &&
            //             next_node->loc.y >= min(node_y, _source->loc.y)) {
            //             // next_node 位于root和node中间，但是线长太长的话也需要继续往下找
            //             bool check = check_WL(node, next_node, eps);
            //             if (check) {
            //                 break;
            //             }
            //             // break;
            //             next_node = *(--find_node);
            //         } else {
            //             next_node = *(--find_node);
            //         }
            //     }
            // }

            next_node = find_ancestor_closest_source(node, tree);
            // next_node = find_father_closest_source_across_path(node, tree);
            // 建立当前点和next_node的边，然后从原本 当前点和next_node 之间移除一条边
            // cout << "node: " << node->id << " best neighbor " << next_node->id << endl;
            assert(next_node);

            // 记录优化前的PL
            int pre_PL = std::accumulate(_length_to_source.begin(), _length_to_source.begin() + _pin_num, 0);
            // int pre_PL = *std::max_element(_length_to_source.begin(), _length_to_source.begin() + _pin_num);
            // pre_PL = calc_avg_stretch();

            shared_ptr<salt::TreeNode> best_remove_edge_1 = nullptr;
            shared_ptr<salt::TreeNode> best_remove_edge_2 = nullptr;
            DTYPE changed_best_WL = INT_MAX;  // 边删除过程中最好的WL

            // 比较_length_to_source和_length_Mah_source对应的值，记录_length_to_source[i] <= _length_Mah_source[i]的点
            set<int> unviolate_node_before;
            for (int i = 0; i < _pin_num; i++) {
                if (_length_to_source[i] <= _eps * _length_Mah_source[i]) {
                    unviolate_node_before.insert(i);
                }
            }

            // 原本 当前点和next_node 之间的路径，因为一直往父节点找，所以可以直接前向遍历找
            auto path = find_path1(node, next_node);
            for (int edge_id = 0; edge_id < path.size() - 1; edge_id++) {  // 找到需要移除哪条边
                auto node_1 = path[edge_id];
                // auto node_2 = node_1->parent;
                auto node_2 = path[edge_id + 1];
                if (!node_2) {
                    continue;
                }

                // 把（node_1，node_2）这条边移除的cost
                DTYPE add_edge_length = utils::Dist(node->loc, next_node->loc);
                DTYPE remove_edge_length = utils::Dist(node_1->loc, node_2->loc);
                DTYPE changed_WL = current_WL + (add_edge_length - remove_edge_length);
                // cout << "remove edge " << node_1->id << ", " << node_2->id << endl;
                // if (changed_WL < eps * _best_WL) {  // 保证WL上界
                    // 计算detour cost

                    // 更新每个点到source的长度,只需要更新部分
                    _length_to_source[node->id] = _length_to_source[next_node->id] + add_edge_length;

                    auto updat_path = find_path1(node, node_2);

                    for (int i = 1; i < updat_path.size() - 1; i++) {
                        auto n1 = path[i];
                        auto n2 = path[i - 1];
                        // cout <<"\t" << n1->id << ", " << n2->id << endl;
                        _length_to_source[n1->id] = _length_to_source[n2->id] + utils::Dist(n1->loc, n2->loc);
                        // n1的子节点也要更新
                        auto childs = n1->children;
                        for (auto child : childs) {
                            if (child->id == n2->id) {
                                continue;
                            }
                            _length_to_source[child->id] = _length_to_source[n1->id] + utils::Dist(child->loc, n1->loc);
                            update_length(child);
                        }
                    }
                    // TODO: 原本满足pl的节点被恶化了，就得continue 
                    // 比较_length_to_source和_length_Mah_source, 
                    set<int> violate_node_after;
                    for (int i = 0; i < _pin_num; i++) {
                        if (_length_to_source[i] > _eps * _length_Mah_source[i]) {
                            violate_node_after.insert(i);
                        }
                    }
                    // violate_node_after中不能包含unviolate_node_before中的点
                    set<int> violate_node;
                    set_intersection(violate_node_after.begin(),
                                   violate_node_after.end(),
                                   unviolate_node_before.begin(),
                                   unviolate_node_before.end(),
                                   inserter(violate_node, violate_node.begin()));
                    if (!violate_node.empty()) {
                        _length_to_source = length_to_source;
                        continue;
                    }
                    // if (violate_node_before != violate_node_after) {
                    //     _length_to_source = length_to_source;
                    //     continue;
                    // }


                    auto post_PL = std::accumulate(_length_to_source.begin(), _length_to_source.begin() + _pin_num, 0);
                    // auto post_PL = *std::max_element(_length_to_source.begin(), _length_to_source.begin() +
                    // _pin_num); if (post_PL < pre_PL && post_PL <= best_PL)
                    // post_PL = calc_avg_stretch();
                    if ((post_PL < pre_PL && post_PL < best_PL) ||
                        (post_PL == best_PL && changed_WL < changed_best_WL) ||
                        (post_PL < pre_PL &&changed_WL < changed_best_WL) ||
                        (post_PL - best_PL) < (changed_best_WL - changed_WL)) {
                        // if ((changed_WL < changed_best_WL)){
                        best_PL = post_PL;
                        best_remove_edge_1 = node_1;
                        best_remove_edge_2 = node_2;
                        changed_best_WL = changed_WL;
                    }
                    _length_to_source = length_to_source;
                // }
            }                                                // 找到需要移除哪条边
            if (best_remove_edge_1 && best_remove_edge_2) {  // 找到了
                // assert(best_remove_edge_1->parent == best_remove_edge_2);
                // TreeNode::resetParent(best_remove_edge_1);
                // TreeNode::setParent(path[i], path[i + 1]);
                // cout << tree;
                // log() << "======== Remove edge: " << best_remove_edge_1->id << ", " << best_remove_edge_2->id << endl;
                if (best_remove_edge_1->parent == best_remove_edge_2) {
                    Flip_order(best_remove_edge_1, node, next_node);
                } else {
                    // Flip_order(node, best_remove_edge_2, next_node);
                    Flip_order(best_remove_edge_2, next_node, node);
                }
                improve = true;
                _current_WL = changed_best_WL;

                // function<void(const shared_ptr<TreeNode>&, DTYPE)> traverse = [&](const shared_ptr<TreeNode>& node,
                //                                                                   DTYPE curDist) {
                //     _length_to_source[node->id] = curDist;
                //     auto node_to_source_manh_dis = utils::Dist(_source->loc, node->loc);
                //     _length_Mah_source[node->id] = node_to_source_manh_dis;
                //     for (auto c : node->children) traverse(c, curDist + c->WireToParent());
                // };
                // traverse(tree.source, 0);
            }
        // }
    }
    return improve;
}

int SelfBuilder::edge_replace_cost(shared_ptr<salt::TreeNode> add_1,
                                   shared_ptr<salt::TreeNode> add_2,
                                   shared_ptr<salt::TreeNode> rem_1,
                                   shared_ptr<salt::TreeNode> rem_2) {}

vector<shared_ptr<TreeNode>> SelfBuilder::find_path(shared_ptr<TreeNode> s_node, shared_ptr<TreeNode> e_node) {
    // 找到路径上的所有点
    std::vector<shared_ptr<TreeNode>> path;
    auto current_node = s_node;

    do {
        path.push_back(current_node);
        if (current_node->id == e_node->id) {
            break;
        }
        current_node = current_node->parent;
    } while (current_node);

    return path;
}
vector<shared_ptr<TreeNode>> SelfBuilder::find_path1(shared_ptr<TreeNode> s_node, shared_ptr<TreeNode> e_node) {
    // 找到路径上的所有点
    std::vector<shared_ptr<TreeNode>> path;
    auto current_node = s_node;

    vector<bool> visited(2 * _pin_num, false);

    do {
        path.push_back(current_node);
        visited[current_node->id] = true;
        if (current_node->id == e_node->id) {
            break;
        }
        current_node = current_node->parent;
    } while (current_node);

    if (!current_node) {
        std::vector<shared_ptr<TreeNode>> path1;
        shared_ptr<TreeNode> common_ancestor = e_node;
        path1.push_back(e_node);
        while (!visited[common_ancestor->id]) {
            common_ancestor = common_ancestor->parent;
            path1.push_back(common_ancestor);
        }
        // do
        // {
        //     path1.push_back(common_ancestor);
        //     common_ancestor = common_ancestor->parent;
        // } while (!visited[common_ancestor->id]);

        while (1) {
            auto n = path.back();
            path.pop_back();
            if (n == common_ancestor) {
                break;
            }
        }
        std::reverse(path1.begin(), path1.end());
        // 将path1的元素插入到path的末尾
        path.insert(path.end(), path1.begin(), path1.end());
    }

    // if (!current_node) {
    //     path.pop_back();
    //     dfs(_source, e_node, path);
    // }
    // auto newEnd = std::unique(path.begin(), path.end(),
    //                           [](const std::shared_ptr<TreeNode>& a, const std::shared_ptr<TreeNode>& b) {
    //                               return *a == *b;
    //                           });

    // // 使用 erase 删除重复元素之间的内容
    // path.erase(newEnd, path.end());

    // auto first = std::adjacent_find(path.begin(), path.end(), [](const std::shared_ptr<TreeNode>& a, const
    // std::shared_ptr<TreeNode>& b) {
    //                               return *a == *b;
    //                           });

    // if (first != path.end()) {
    //     // 找到第二个相同元素的位置
    //     auto second = std::adjacent_find(first + 1, path.end());

    //     if (second != path.end()) {
    //         // 删除两个相同元素之间的值
    //         path.erase(first + 1, second);
    //     }
    // }

    return path;
}

void SelfBuilder::Flip_order(shared_ptr<TreeNode> p_node,
                             shared_ptr<TreeNode> c_node,
                             shared_ptr<TreeNode> c_node_father) {
    // // 找到路径上的所有点
    // std::vector<shared_ptr<TreeNode>> path;
    // auto current_node = c_node;

    // do
    // {
    //     path.push_back(current_node);
    //     if (current_node->id == p_node->id) {
    //         break;
    //     }
    //     current_node = current_node->parent;
    // } while (current_node);

    auto path = find_path1(c_node, p_node);

    std::reverse(path.begin(), path.end());

    // for (auto n : path) {
    //     cout << n->id << ", ";
    // }
    // cout << endl;

    auto path_length = path.size();

    for (size_t i = 0; i < path_length - 1; ++i) {
        // path[i]->setParent(path[i - 1]);
        // path[i - 1]->addChild(path[i]);
        TreeNode::resetParent(path[i]);
        TreeNode::setParent(path[i], path[i + 1]);
    }
    TreeNode::resetParent(path[path_length - 1]);
    TreeNode::setParent(path[path_length - 1], c_node_father);
}

// bool SaltBuilder::Relax(const shared_ptr<TreeNode>& u, const shared_ptr<TreeNode>& v) {
//     DTYPE newDist = curDists[u->id] + Dist(u->loc, v->loc);
//     if (curDists[v->id] > newDist) {
//         curDists[v->id] = newDist;
//         v->parent = u;
//         return true;
//     } else if (curDists[v->id] == newDist && Dist(u->loc, v->loc) < v->WireToParentChecked()) {
//         v->parent = u;
//         return true;
//     } else
//         return false;
// }

// void SaltBuilder::DFS(const shared_ptr<TreeNode>& smtNode, const shared_ptr<TreeNode>& slNode, double eps) {
//     if (smtNode->pin && curDists[slNode->id] > (1 + eps) * shortestDists[slNode->id]) {
//         slNode->parent = slSrc;
//         curDists[slNode->id] = shortestDists[slNode->id];
//     }
//     for (auto c : smtNode->children) {
//         Relax(slNode, slNodes[c->id]);
//         DFS(c, slNodes[c->id], eps);
//         Relax(slNodes[c->id], slNode);
//     }
// }

void SelfBuilder::post_processing1(Tree& tree) {
    auto all_node = tree.ObtainNodes();
    for (auto node : all_node) {
        // cout << "cdjbajd " << node->id << endl;
        bool improve = false;

        // if (node->id != 22) {
        //     cout << endl;
        //     continue;
        // }
        Point node_loc, p_loc, pp_loc, c_loc;
        auto p = node->parent;
        if (p == nullptr) continue;
        auto pp = p->parent;
        while (pp && pp->children.size() != 2) {
            pp = pp->parent;
            if (pp == nullptr) break;
        }
        if (pp == nullptr) continue;

        node_loc = node->loc;
        p_loc = p->loc;
        pp_loc = pp->loc;

        while (!improve) {
            pp = pp->parent;
            while (pp && pp->children.size() != 2) {
                pp = pp->parent;
                if (pp == nullptr) break;
            }
            if (pp == nullptr) break;
            pp_loc = pp->loc;
            // if (pp->children.size() == 2) {
            // pp 的另一个孩子
            bool x_equ = (pp->children[0]->loc.x == p->loc.x);
            bool y_equ = (pp->children[0]->loc.y == p->loc.y);
            auto c = (x_equ || y_equ) ? pp->children[1] : pp->children[0];
            assert(c->id != p->id);
            c_loc = c->loc;

            // 判断（p,pp）是垂直边(d=0)还是平行边(d=1)
            int d;  // primary direction (0-x: in[0]-in[1] is vertical; 1-y)
            for (d = 0; d < 2; ++d) {
                if (p_loc[d] == pp_loc[d]) break;
            }
            if (d == 2) continue;  // not aligned

            if (pp_loc[d] == c_loc[d]) continue;

            if (pp_loc[1 - d] != c_loc[1 - d]) continue;  // 新加的

            bool larger;  // out[i] is in the larger side of in[i] (in secondary direction)
            larger = pp_loc[d] > c_loc[d];

            if (node_loc[d] == p_loc[d]) continue;
            if ((pp_loc[d] > node_loc[d]) != larger) continue;  // should be at the same side

            DTYPE closer = larger ? max(node_loc[d], c_loc[d]) : min(node_loc[d], c_loc[d]);

            Point new_point;
            if (d == 0) {
                new_point = Point(closer, pp_loc[1]);
            } else {
                new_point = Point(pp_loc[0], closer);
            }

            int node_pl = _length_to_source[node->id];

            int new_point_pl = _length_to_source[pp->id] + utils::Dist(pp_loc, new_point);
            int node_to_p_dis = utils::Dist(p_loc, node_loc);
            int a = abs((p_loc[1 - d] - pp_loc[1 - d]));
            int b = abs((new_point[d] - node_loc[d]));
            int node_to_edge_pp_c_dis = abs(p_loc[1 - d] - pp_loc[1 - d]) + abs(new_point[d] - node_loc[d]);
            node_to_edge_pp_c_dis = a + b;

            int node_pl_new = new_point_pl + utils::Dist(node_loc, new_point);
            int node_to_source_Mah_dis = utils::Dist(node_loc, _source->loc);
            // PL依然满足条件，并且线长降低
            if (node_pl_new < _eps * node_to_source_Mah_dis && node_to_edge_pp_c_dis < node_to_p_dis) {
                // cout << node->id << "(" << node_loc.x << ", " << node_loc.y << ")"
                //      << "->" << p->id << "(" << p_loc.x << ", " << p_loc.y << ")"
                //      << "->" << pp->id << "(" << pp_loc.x << ", " << pp_loc.y << ")"
                //      << "->" << c->id << "(" << c_loc.x << ", " << c_loc.y << ")" << endl;
                // cout << node_to_edge_pp_c_dis << "   " << INT_MAX << endl;
                auto new_node = make_shared<TreeNode>(new_point);
                TreeNode::resetParent(node);
                TreeNode::resetParent(c);
                TreeNode::setParent(new_node, pp);
                TreeNode::setParent(node, new_node);
                TreeNode::setParent(c, new_node);
                // return;
                improve = true;
                break;
            }
        }
    }
    tree.RemoveTopoRedundantSteiner();
}

void SelfBuilder::post_processing(Tree& tree) {
    auto all_node = tree.ObtainNodes();
    for (auto node : all_node) {
        // if (node->id == 39) {
        //     cout << endl;
        // }
        Point node_loc, p_loc, pp_loc, c_loc;
        auto p = node->parent;
        if (p == nullptr) continue;
        auto pp = p->parent;
        while (pp && pp->children.size() < 2) {
            pp = pp->parent;
            if (pp == nullptr) break;
        }
        if (pp == nullptr) continue;

        node_loc = node->loc; 
        p_loc = p->loc;
        pp_loc = pp->loc;

        if (pp->children.size() > 1) {
            // pp 的另一个孩子
            for (int i = 0; i < pp->children.size(); i++) {
                // auto c = (pp->children[0] == p) ? pp->children[1] : pp->children[0];
                if (pp->children[i] == p) continue;
                auto c = pp->children[i];
                assert(c->id != p->id);
                c_loc = c->loc;

                // 判断（p,pp）是垂直边(d=0)还是平行边(d=1)
                int d;  // primary direction (0-x: in[0]-in[1] is vertical; 1-y)
                for (d = 0; d < 2; ++d) {
                    if (p_loc[d] == pp_loc[d]) break;
                }
                if (d == 2) continue;  // not aligned

                // TODO: c_loc 和pp_loc 可能不是水平/垂直线

                if (pp_loc[d] == c_loc[d]) continue;
                if (node_loc[d] == p_loc[d]) continue;

                // if (pp_loc[1 - d] != c_loc[1 - d]) continue;  // 新加的

                bool larger;  // out[i] is in the larger side of in[i] (in secondary direction)
                larger = pp_loc[d] > c_loc[d];

                if ((pp_loc[d] > node_loc[d]) != larger) continue;  // should be at the same side

                DTYPE closer = larger ? max(node_loc[d], c_loc[d]) : min(node_loc[d], c_loc[d]);

                auto another_closer =
                    pp_loc[1 - d] > p_loc[1 - d] ? min(pp_loc[1 - d], c_loc[1 - d]) : max(pp_loc[1 - d], c_loc[1 - d]);

                Point new_point;
                if (d == 0) {
                    // new_point = Point(closer, pp_loc[1]);
                    new_point = Point(closer, another_closer);
                } else {
                    // new_point = Point(pp_loc[0], closer);
                    new_point = Point(another_closer, closer);
                }

                int node_pl = _length_to_source[node->id];

                int new_point_pl = _length_to_source[pp->id] + utils::Dist(pp_loc, new_point);
                int node_to_p_dis = utils::Dist(p_loc, node_loc);
                int a = 0;//abs((p_loc[1 - d] - pp_loc[1 - d]));
                int b = utils::Dist(node_loc, new_point);
                int node_to_edge_pp_c_dis = abs(p_loc[1 - d] - pp_loc[1 - d]) + abs(new_point[d] - node_loc[d]);
                node_to_edge_pp_c_dis = a + b;

                int node_pl_new = new_point_pl + utils::Dist(node_loc, new_point);
                int node_to_source_Mah_dis = utils::Dist(node_loc, _source->loc);
                // PL依然满足条件，并且线长降低
                if (node_pl_new <= _eps * node_to_source_Mah_dis && node_to_edge_pp_c_dis <= node_to_p_dis) {
                    // cout << node->id << "(" << node_loc.x << ", " << node_loc.y << ")"
                    //     << "->" << p->id << "(" << p_loc.x << ", " << p_loc.y << ")"
                    //     << "->" << pp->id << "(" << pp_loc.x << ", " << pp_loc.y << ")"
                    //     << "->" << c->id << "(" << c_loc.x << ", " << c_loc.y << ")" << endl;
                    // cout << node_to_edge_pp_c_dis << "   " << INT_MAX << endl;
                    auto new_node = make_shared<TreeNode>(new_point, nullptr, _make_steiner_id);
                    _make_steiner_id++;
                    TreeNode::resetParent(node);
                    TreeNode::resetParent(c);
                    TreeNode::setParent(new_node, pp);
                    TreeNode::setParent(node, new_node);
                    TreeNode::setParent(c, new_node);
                    // return;
                }
            }
        }
    }
    tree.RemoveTopoRedundantSteiner();
}

}  // namespace salt
