#pragma once

#include "tree.h"
#include <iostream>
#include <iomanip>
#include <fstream>
namespace salt {

class WireLengthEvalBase {
public:
    DTYPE wireLength;

    WireLengthEvalBase() = default;
    void Update(const Tree& tree);
    WireLengthEvalBase(const Tree& tree) { Update(tree); }
};

class WireLengthEval : public WireLengthEvalBase {
public:
    DTYPE maxPathLength;
    double avgPathLength;
    double norPathLength;  // avgPathLength / avgShortestPathLength
    double maxStretch;     // max{pathLength / shortestPathLength}
    double avgStretch;     // avg{pathLength / shortestPathLength}
    int max_path_length_id = -1;
    vector<int> nodeLevel;

    WireLengthEval() = default;
    void Update(const Tree& tree);
    WireLengthEval(const Tree& tree) { Update(tree); }
};

inline ostream& operator<<(ostream& os, const WireLengthEval& eval) {
    os << " wl=" << eval.wireLength << " mp=" << eval.maxPathLength << " ap=" << eval.avgPathLength
       << " ms=" << eval.maxStretch << " as=" << eval.avgStretch;
    return os;
}

//********************************************************************************

class ElmoreDelayEval {
public:
    static double unitRes;
    static double unitCap;

    DTYPE fluteWL = -1;
    double maxDelay;
    double avgDelay;
    double sumDelay;
    double sumNorDelay;
    double maxNorDelay;
    double avgNorDelay;
    int max_delay_node_id = -1;
    int max_delay_node_level = -1;
    vector<double> nodeDelay;
    vector<double> nodeNorDelay;

    ElmoreDelayEval() {}
    void Calc(double rd, Tree& tree);   // 不是包含所有pin
    void Update(double rd, Tree& tree, bool normalize = true);  // tree node id will be updated
    ElmoreDelayEval(double rd, Tree& tree, bool normalize = true) { Update(rd, tree, normalize); }

    double _maxLb = 0.0;  // 对每条net，每次都要重新计算，所以缓存一下，这个值只对一条net是不变的
    ElmoreDelayEval(double rd, Tree& tree, double maxLb) {
        assert(rd > 0);
        assert(unitRes > 0 && unitCap > 0);
        int numPins = tree.net->pins.size();
        int numNodes = tree.ObtainNodes().size();
        maxDelay = avgDelay = maxNorDelay = avgNorDelay = sumDelay = sumNorDelay = 0;
        auto delay = GetDelay(rd, tree, numNodes);  // delay for all tree nodes
        nodeDelay = delay;
        tree.preOrder([&](const shared_ptr<TreeNode>& node) {
            if (!node->pin || node == tree.source) return;
            sumDelay += delay[node->id];
        });
        sumNorDelay = sumDelay / maxLb;

        nodeNorDelay.resize(numNodes, 0);
        for (int i = 0; i < numNodes; i++) {
            nodeNorDelay[i] = nodeDelay[i] / maxLb;
        }
    }

private:
    vector<double> GetDelay(double rd, const Tree& tree, int numNode);
    vector<double> GetDelayLB(double rd, const Tree& tree);
};

inline ostream& operator<<(ostream& os, const ElmoreDelayEval& eval) {
    os << " md=" << eval.maxDelay << " ad=" << eval.avgDelay << " mnd=" << eval.maxNorDelay
       << " and=" << eval.avgNorDelay;
    return os;
}

//********************************************************************************

class CompleteEval : public WireLengthEval, public ElmoreDelayEval {
public:
    double norWL;

    CompleteEval() = default;
    void Update(double rd, Tree& tree) {
        WireLengthEval::Update(tree);
        ElmoreDelayEval::Update(rd, tree);
        norWL = double(wireLength) / fluteWL;
    }
    CompleteEval(double rd, Tree& tree) { Update(rd, tree); }
};

class CompleteStat {
public:
    double norWL = 0, maxStretch = 0, avgStretch = 0, norPathLength = 0, maxNorDelay = 0, avgNorDelay = 0;
    vector<double> WL;
    int cnt = 0;
    double eps;
    double time = 0;
    void Inc(const CompleteEval& eval, double runtime = 0.0) {
        WL.push_back(eval.norWL);
        ++cnt;
        norWL += eval.norWL;
        maxStretch += eval.maxStretch;
        avgStretch += eval.avgStretch;
        norPathLength += eval.norPathLength;
        maxNorDelay += eval.maxNorDelay;
        avgNorDelay += eval.avgNorDelay;
        time += runtime;
    }
    void Avg() {
        norWL /= cnt;
        maxStretch /= cnt;
        avgStretch /= cnt;
        norPathLength /= cnt;
        maxNorDelay /= cnt;
        avgNorDelay /= cnt;
        time /= cnt;
    }
};

class TimeStat {
public:
    double time = 0;
    int cnt = 0;
    double change_topo_time = 0;
    int change_topo_cnt = 0;
    double recover_topo_time = 0;
    int recover_topo_cnt = 0;
    double calc_delay_time = 0;
    int calc_delay_cnt = 0;

    void Inc_change_topo(double runtime) {
        change_topo_time += runtime;
        ++change_topo_cnt;
    }
    void Inc_recover_topo(double runtime) {
        recover_topo_time += runtime;
        ++recover_topo_cnt;
    }
    void Inc_calc_delay(double runtime) {
        calc_delay_time += runtime;
        ++calc_delay_cnt;
    }
    void Avg() { time /= cnt; }
};

}  // namespace salt