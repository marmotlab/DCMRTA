#ifndef TEAMPLANNER_GRAPH_HPP
#define TEAMPLANNER_GRAPH_HPP

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>

class GraphEdge
{
public:
    int node1_;
    int node2_;
    int type_;
public:
    GraphEdge();
    GraphEdge(int node1, int node2, int type);
    ~GraphEdge();

    inline void set(int node1, int node2, int type) {
        node1_ = node1;
        node2_ = node2;
        type_ = type;
    }

    inline friend std::ostream& operator<<(std::ostream& os, const GraphEdge& edge)
    {
        os << edge.toString();
        return os;
    }

    inline bool operator==(const GraphEdge& rhs) const
    {
        return ( (node1_ == rhs.node1_) && (node2_ == rhs.node2_) && (type_ == rhs.type_) );
    }

    inline std::string toString() const {
        return ( std::to_string(node1_) + "," + std::to_string(node2_) + "," + std::to_string(type_) );
    }

};

struct GraphEdgeComparator
{
    inline bool operator()(const GraphEdge& lhs, const GraphEdge& rhs) const
    {
        return lhs == rhs;
    }
};

struct GraphEdgeHasher
{
    inline size_t operator()(const GraphEdge& edge) const
    {
        return std::hash<std::string>{}(edge.toString());
    }
};

class GraphEdgeParam
{
public:
    GraphEdge edge_;
    double engCost_;
    double engUnc_;
    double timeCost_;
public:
    GraphEdgeParam();
    GraphEdgeParam(const GraphEdge& edge, double engCost, double engUnc, double timeCost);
    ~GraphEdgeParam();
    inline void remove() {
        edge_.set(-1, -1, 0);
    }
    inline bool exist() const {
        return (edge_.node1_ >= 0);
    }
};

class GraphNodeParam
{
public:
    int node_;
    double timeCost_;
    std::unordered_set<int> outEdgeIds_;
    std::unordered_set<int> inEdgeIds_;
public:
    GraphNodeParam();
    GraphNodeParam(int node, double timeCost);
    ~GraphNodeParam();
    inline void remove() {
        node_ = -1;
        outEdgeIds_.clear();
        inEdgeIds_.clear();
    }
    inline bool exist() const {
        return (node_ >= 0);
    }
};


class Graph
{
protected:
    std::unordered_map<int, int> node2nodeId_;
    std::vector<GraphNodeParam> nodeId2nodeParam_;
    std::unordered_map<GraphEdge, int, GraphEdgeHasher, GraphEdgeComparator> edge2edgeId_;
    std::vector<GraphEdgeParam> edgeId2edgeParam_;
public:
    Graph();
    ~Graph();
    void clear();
    bool addEdge(int node1, int node2, int type, double engCost, double engUnc, double timeCost, bool flagDirected = false);
    bool addNode(int node, double timeCost);
    void print() const;
    int edge2id(int node1, int node2, int type) const;
    bool id2edge(int id, GraphEdgeParam& edgeParam) const;
    int node2id(int node) const;
    bool id2node(int id, GraphNodeParam& nodeParam) const;
    const GraphNodeParam& node(int id) const;
    const GraphEdgeParam& edge(int id) const;
    bool readFromFile(const std::string& fileName, const std::string& yamlPrefix = "", int nodeNum = -1);
    inline size_t edgeNum() const {
        return edge2edgeId_.size();
    }
    inline size_t nodeNum() const {
        return node2nodeId_.size();
    }
    inline bool hasEdge(int node1, int node2, int type) const {
        GraphEdge tempEdge(node1, node2, type);
        return (edge2edgeId_.find(tempEdge) != edge2edgeId_.end());
    }
    inline bool hasEdge(const GraphEdge& tempEdge) const {
        return (edge2edgeId_.find(tempEdge) != edge2edgeId_.end());
    }
    inline bool hasNode(int node) const {
        return (node2nodeId_.find(node) != node2nodeId_.end());
    }
};

#endif //TEAMPLANNER_GRAPH_HPP