#ifndef FLOWCONVERTER_HPP
#define FLOWCONVERTER_HPP

#include <gurobi_c++.h>
#include <Graph.hpp>
#include <string>

class FlowConverter
{
public:
    GRBEnv* env_;
    GRBModel* roundModel_;
    GRBModel* coverModel_;
    Graph graph_;
    GRBVar* intFlowVar_;
    GRBVar* coverFlowVar_;
    GRBVar vehVar_;
    int edgeNum_;
    int nodeNum_;
    int startNode_;
    int endNode_;
    int intFlow_;
    double oriFlow_;
    bool roundOptimized_;
    bool coverOptimized_;
    int verboseLevel_;
public:
    FlowConverter(/* args */);
    ~FlowConverter();
    void clear();
    void initializeEnv();
    std::string offsetString(int type, int id, bool flagOffset) const;
    std::vector<int> offsetString(int type, const std::vector<int>& id, bool flagOffset) const;
    // Round
    bool formRoundProblem(const std::string& paramFile);
    bool formRoundProblem(int startNode, int endNode);
    bool formRoundProblem();
    void initializeRoundModel();
    void formRoundConstraints(int startNode, int endNode);
    std::string toStringRound(const std::string& prefix = "") const;
    void printRound() const;
    void saveRound(const std::string& saveFileName, const std::string& prefix = "") const;
    bool optimizeRound(double solverMaxTime = -1.0);
    // Cover
    void initializeCoverModel();
    bool getFlow();
    void formCoverConstraints();
    bool formCoverProblem();
    bool optimizeCover(double solverMaxTime = -1.0);
    std::string toStringCover(bool flagSave, const std::string& prefix = "") const;
    void getPath(std::vector<std::vector<int>>& vehPath, std::vector<std::vector<int>>& vehEdge, std::vector<double>& vehSumEng, std::vector<double>& vehSumVar, const double* xValue = nullptr) const;
    void printCover() const;
    void saveCover(const std::string& saveFileName, const std::string& prefix = "") const;

    inline int cover2id(int veh, int edge) const {
        return (veh * edgeNum_ + edge);
    }
    inline void id2cover(int id, int& veh, int& edge) const {
        veh = id / edgeNum_;
        edge = id - veh * edgeNum_;
    }
    inline void setVerboseLevel(int verboseLevel) {
        verboseLevel_ = verboseLevel;
    }

};



#endif //FLOWCONVERTER_HPP