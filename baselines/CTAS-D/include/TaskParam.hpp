#ifndef TEAMPLANNER_TASKPARAM_HPP
#define TEAMPLANNER_TASKPARAM_HPP

#include <vector>
#include <string>

class TaskReqGeq
{
public:
    bool geq_;
    int capId_;
    double capReq_;
    double capVar_;
public:
    TaskReqGeq();
    TaskReqGeq(int capId, double capReq, double capVar, bool geq = true);
    ~TaskReqGeq();
    std::string toString() const;
};

class TaskParam
{
public:
    std::vector<std::vector<TaskReqGeq>> reqFcn_;
public:
    TaskParam();
    ~TaskParam();
    std::string toString() const;
};

#endif //TEAMPLANNER_TASKPARAM_HPP