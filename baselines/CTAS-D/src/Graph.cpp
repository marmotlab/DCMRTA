#include <Graph.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <yaml-cpp/yaml.h>

GraphEdge::GraphEdge() {
    set(-1, -1, 0);
}

GraphEdge::GraphEdge(int node1, int node2, int type) {
    set(node1, node2, type);
}

GraphEdge::~GraphEdge() {
}

GraphEdgeParam::GraphEdgeParam() {
    engCost_ = 0.0;
    engUnc_ = 0.0;
    timeCost_ = 0.0;
}

GraphEdgeParam::GraphEdgeParam(const GraphEdge& edge, double engCost, double engUnc, double timeCost) {
    edge_ = edge;
    engCost_ = engCost;
    engUnc_ = engUnc;
    timeCost_ = timeCost;
}

GraphEdgeParam::~GraphEdgeParam() {
}

GraphNodeParam::GraphNodeParam() {
    node_ = -1;
    timeCost_ = 0.0;
}

GraphNodeParam::GraphNodeParam(int node, double timeCost) {
    node_ = node;
    timeCost_ = timeCost;
}

GraphNodeParam::~GraphNodeParam() {    
}

Graph::Graph() {
}

Graph::~Graph() {
}

void Graph::clear() {
    edge2edgeId_.clear();
    node2nodeId_.clear();
    edgeId2edgeParam_.clear();
    nodeId2nodeParam_.clear();
}

bool Graph::addEdge(int node1, int node2, int type, double engCost, double engUnc, double timeCost, bool flagDirected) {
    // Not checking existence of the edge to add
    if (!hasNode(node1)) {
        fprintf(stderr, "ERROR: No such graph node.\n");
        return false;
    }
    if (!hasNode(node2)) {
        fprintf(stderr, "ERROR: No such graph node.\n");
        return false;
    }
    GraphEdge tempEdge(node1, node2, type);
    if (hasEdge(tempEdge)) {
        return false;
    }
    GraphEdgeParam tempEdgeParam(tempEdge, engCost, engUnc, timeCost);
    int tempEdgeId = static_cast<int>(edgeId2edgeParam_.size());
    edgeId2edgeParam_.push_back(tempEdgeParam);
    edge2edgeId_[tempEdge] = tempEdgeId;
    int node1Id = node2nodeId_[node1];
    int node2Id = node2nodeId_[node2];
    nodeId2nodeParam_[node1Id].outEdgeIds_.insert(tempEdgeId);
    nodeId2nodeParam_[node2Id].inEdgeIds_.insert(tempEdgeId);
    if (flagDirected) {
        return true;
    }
    tempEdge.node1_ = node2;
    tempEdge.node2_ = node1;
    tempEdgeParam.edge_ = tempEdge;
    tempEdgeId = static_cast<int>(edgeId2edgeParam_.size());
    edgeId2edgeParam_.push_back(tempEdgeParam);
    edge2edgeId_[tempEdge] = tempEdgeId;
    nodeId2nodeParam_[node2Id].outEdgeIds_.insert(tempEdgeId);
    nodeId2nodeParam_[node1Id].inEdgeIds_.insert(tempEdgeId);
    return true;
}

bool Graph::addNode(int node, double timeCost) {
    // Checking existence of the node to add
    if (hasNode(node)) {
        return false;
    }
    GraphNodeParam tempNodeParam(node, timeCost);
    int tempNodeId = static_cast<int>(nodeId2nodeParam_.size());
    nodeId2nodeParam_.push_back(tempNodeParam);
    node2nodeId_[node] = tempNodeId;
    return true;
}

void Graph::print() const {
    std::cout << "Node:\n";
    for (size_t i = 0; i < nodeId2nodeParam_.size(); i++) {
        std::cout << nodeId2nodeParam_[i].node_ << ";   ";
    }
    std::cout << "\nEdge:\n";
    for (size_t i = 0; i < edgeId2edgeParam_.size(); i++) {
        std::cout << edgeId2edgeParam_[i].edge_ << ";   ";
    }
    std::cout <<"\n";
}

int Graph::edge2id(int node1, int node2, int type) const {
    GraphEdge tempEdge(node1, node2, type);
    auto edgePtr = edge2edgeId_.find(tempEdge);
    if (edgePtr != edge2edgeId_.end()) {
        return edgePtr->second;
    }
    return -1;
}
bool Graph::id2edge(int id, GraphEdgeParam& edgeParam) const {
    if (id < static_cast<int>(edgeNum()) && edgeId2edgeParam_[id].exist() ) {
        edgeParam = edgeId2edgeParam_[id];
        return true;
    }
    return false;
}
int Graph::node2id(int node) const {
    auto nodePtr = node2nodeId_.find(node);
    if (nodePtr != node2nodeId_.end()) {
        return nodePtr->second;
    }
    return -1;
}
bool Graph::id2node(int id, GraphNodeParam& nodeParam) const {
    if (id < static_cast<int>(nodeNum()) && nodeId2nodeParam_[id].exist() ) {
        nodeParam = nodeId2nodeParam_[id];
        return true;
    }
    return false;
}

const GraphNodeParam& Graph::node(int id) const {
    if (id >= static_cast<int>(nodeId2nodeParam_.size()) ) {
        fprintf(stderr, "ERROR: Graph node.\n");
    }
    return nodeId2nodeParam_[id];
}

const GraphEdgeParam& Graph::edge(int id) const {
    if (id >= static_cast<int>(edgeId2edgeParam_.size()) ) {
        fprintf(stderr, "ERROR: Graph edge.\n");
    }
    return edgeId2edgeParam_[id];
}

bool Graph::readFromFile(const std::string& fileName, const std::string& yamlPrefix, int nodeNum) {
    double nodeTimeCost = 1.0;
    try {
        clear();
        // Set data node
        YAML::Node yamlParam = YAML::LoadFile(fileName);
        YAML::Node yamlGraph;
        if (yamlPrefix.compare("") == 0) {
            yamlGraph = yamlParam;
        }
        else {
            yamlGraph = yamlParam[yamlPrefix];
        }
        // Add nodes
        if (nodeNum < 0) {
            nodeNum = yamlGraph["nodeNum"].as<int>();
        }
        for (int nodeId = 0; nodeId < nodeNum; nodeId++) {
            this->addNode(nodeId, nodeTimeCost);
        }
        // Add edges
        for (int edgeId = 0; ; edgeId++) {
            std::string edgeString = "edge" + std::to_string(edgeId);
            if (!yamlGraph[edgeString]) {
                break;
            }
            int node1       = yamlGraph[edgeString][0].as<int>();
            int node2       = yamlGraph[edgeString][1].as<int>();
            int type        = yamlGraph[edgeString][2].as<int>();
            double engCost  = yamlGraph[edgeString][3].as<double>();
            double engUnc   = yamlGraph[edgeString][4].as<double>();
            double timeCost = yamlGraph[edgeString][5].as<double>();
            bool flagDirected = true;
            this->addEdge(node1, node2, type, engCost, engUnc, timeCost, flagDirected);
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        exit(-1);
        return false;
    }
    return true;
}