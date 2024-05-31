#include "eval.h"
#include "flute.h"

#include <algorithm>

namespace salt {

void WireLengthEvalBase::Update(const Tree& tree) {
    wireLength = 0;
    tree.postOrder([&](const shared_ptr<TreeNode>& node) {
        if (node->parent) {
            wireLength += node->WireToParent();
        }
    });
}

void WireLengthEval::Update(const Tree& tree) {
    // wirelength
    WireLengthEvalBase::Update(tree);
    // path
    vector<DTYPE> pathLength(tree.net->pins.size());
    nodeLevel.resize(tree.net->pins.size());
    function<void(const shared_ptr<TreeNode>&, DTYPE)> traverse = [&](const shared_ptr<TreeNode>& node, DTYPE curDist) {
        if (node->pin) {
            pathLength[node->pin->id] = curDist;
            nodeLevel[node->pin->id] = node->level;
        }
        for (auto c : node->children) traverse(c, curDist + c->WireToParent());
    };
    traverse(tree.source, 0);
    maxPathLength = 0;
    double totalPathLength = 0;
    double totalShortestPathLength = 0;
    maxStretch = 0;
    avgStretch = 0;
    for (auto p : tree.net->pins) {
        if (p->IsSink()) {
            DTYPE pl = pathLength[p->id], sp = Dist(tree.source->loc, p->loc);
            double stretch = double(pl) / sp;
            // cout << p->id << " stretch " << stretch << endl;
            if (pl > maxPathLength) {
                // cout << p->id << " " << stretch << endl;
                maxPathLength = pl;
                max_path_length_id = p->id;
            }
            totalPathLength += pl;
            totalShortestPathLength += sp;
            if (stretch > maxStretch) {
                // cout << p->id << " maxStretch " << stretch << endl;
                maxStretch = stretch;}
            avgStretch += stretch;
        }
    }
    auto numSink = tree.net->pins.size() - 1;
    avgPathLength = totalPathLength / numSink;
    norPathLength = totalPathLength / totalShortestPathLength;
    avgStretch /= numSink;
}

//********************************************************************************

double ElmoreDelayEval::unitRes = -1;
double ElmoreDelayEval::unitCap = -1;

void ElmoreDelayEval::Calc(double rd, Tree& tree) {
    assert(rd > 0);
    assert(unitRes > 0 && unitCap > 0);

    // int numPins = 0;
    // tree.preOrder([&](const shared_ptr<TreeNode>& node) {
    //     if (node->pin) {
    //         numPins++;
    //     }
    // });

    // int numNodes = tree.ObtainNodes().size();
    // 设置一个大点的值，防止超出索引
    int numNodes = 5 * tree.net->pins.size();
    maxDelay = avgDelay = maxNorDelay = avgNorDelay = sumDelay = sumNorDelay = 0;

    auto delay = GetDelay(rd, tree, numNodes);  // delay for all tree nodes
    tree.preOrder([&](const shared_ptr<TreeNode>& node) {
        if (!node->pin || node == tree.source) return;
        maxDelay = max(maxDelay, delay[node->id]);
        avgDelay += delay[node->id];
        sumDelay += delay[node->id];
    });
    // avgDelay /= (numPins - 1);
}

void ElmoreDelayEval::Update(double rd, Tree& tree, bool normalize) {
    assert(rd > 0);
    assert(unitRes > 0 && unitCap > 0);
    int numPins = tree.net->pins.size();
    // int numNodes = tree.UpdateId();
    int numNodes = tree.ObtainNodes().size();
    maxDelay = avgDelay = maxNorDelay = avgNorDelay = sumDelay = sumNorDelay = 0;
    nodeDelay.resize(numNodes, 0);
    nodeNorDelay.resize(numNodes, 0);

    // tree.preOrder([&](const shared_ptr<TreeNode>& node){
    // 	if(!node->pin || node == tree.source) return;
    // 	double norDelay = delay[node->id] / lb[node->id];
    // 	maxNorDelay = max(maxNorDelay, norDelay);
    // 	avgNorDelay += norDelay;
    // });
    // avgNorDelay /= (numPins-1);

    auto delay = GetDelay(rd, tree, numNodes);  // delay for all tree nodes
    nodeDelay = delay;
    tree.preOrder([&](const shared_ptr<TreeNode>& node) {
        // nodeDelay[node->id] = delay[node->id];
        // nodeNorDelay[node->id] = delay[node->id] / maxLb;
        if (!node->pin || node == tree.source) return;
        maxDelay = max(maxDelay, delay[node->id]);
        avgDelay += delay[node->id];
        sumDelay += delay[node->id];
        max_delay_node_id = maxDelay == delay[node->id] ? node->id : max_delay_node_id;
        max_delay_node_level = maxDelay == delay[node->id] ? node->level : max_delay_node_level;
    });
    avgDelay /= (numPins - 1);

    if (!normalize) return;

    auto lb = GetDelayLB(rd, tree);  // delay lb for all pins, 0 is source
    auto maxLb = *max_element(lb.begin(), lb.end());
    _maxLb = maxLb;

    for (int i = 0; i < numNodes; i++) {
        nodeNorDelay[i] = nodeDelay[i] / maxLb;
    }
    maxNorDelay = maxDelay / maxLb;
    avgNorDelay = avgDelay / maxLb;
    sumNorDelay = sumDelay / maxLb;
}

vector<double> ElmoreDelayEval::GetDelay(double rd, const Tree& tree, int numNode) {
    auto calc_cap = [&](DTYPE length) {
        double cap = 3.69866e-05*length*0.17+4.0697e-05*2*(length+0.17);//gcd
        // double cap = 0.0002*length*0.05+8.29e-05*2*(length+0.05);//28nm
        // double cap = 0.00362*length*0.16+6.98e-07*2*(length+0.16);//90nm
        return cap;
    };
    auto calc_res = [&](DTYPE length) {
        double res = 12.2*length/0.17;//gcd
        // double res = 0.407*length/0.05;//28nm
        // double res = 0.0995*length/0.16;//90
        return res;
    };

    // get node cap by post-order traversal
    vector<double> cap(numNode, 0);
    tree.postOrder([&](const shared_ptr<TreeNode>& node) {
        if (node->pin && node != tree.source) cap[node->id] = node->pin->cap;
        for (auto c : node->children) {
            cap[node->id] += cap[c->id];
            cap[node->id] += calc_cap(c->WireToParent()/1000);//gcd/90
            // cap[node->id] += calc_cap(c->WireToParent()/2000);//28nm
            // cap[node->id] += c->WireToParent() * unitCap;
        }
    });

    // get delay by post-order traversal
    vector<double> delay(numNode, 0);
    tree.preOrder([&](const shared_ptr<TreeNode>& node) {
        if (node == tree.source)
            delay[node->id] = rd * cap[node->id];
        else {
            double dist = node->WireToParent();
            // delay[node->id] = dist * unitRes * (0.5 * dist * unitCap + cap[node->id]) + delay[node->parent->id];
            delay[node->id] = calc_res(dist/1000) * (0.5 * calc_cap(dist/1000) + cap[node->id]) + delay[node->parent->id];// gcd/90
            // delay[node->id] = calc_res(dist/2000) * (0.5 * calc_cap(dist/2000) + cap[node->id]) + delay[node->parent->id];
        }
    });
    return delay;
}

vector<double> ElmoreDelayEval::GetDelayLB(double rd, const Tree& tree) {
    vector<double> lb(tree.net->pins.size(), 0);

    // call flute and get smt
    Tree flute;
    FluteBuilder fluteB;
    fluteB.Run(*tree.net, flute);
    WireLengthEvalBase wl(flute);
    fluteWL = wl.wireLength;

    double totalcap = 0;
    for (auto p : tree.net->pins) totalcap += p->cap;

    double lb_sd = rd * (fluteWL * unitCap + totalcap);
    for (auto pin : tree.net->pins) {
        if (pin->IsSource()) continue;
        double dist = Dist(tree.source->loc, pin->loc);
        lb[pin->id] = dist * unitRes * (0.5 * dist * unitCap + pin->cap) + lb_sd;
    }

    return lb;
}

}  // namespace salt