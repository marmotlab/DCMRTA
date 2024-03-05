#ifndef TEAMPLANNER_PY_HPP
#define TEAMPLANNER_PY_HPP

#ifndef PY_SSIZE_T_CLEAN
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#endif

#include <vector>
#include <string>
#include <TeamPlanner.hpp>
// class TeamPlanner;

class TeamPlannerPyWrapper
{
private:
    TeamPlanner planner_;
public:
    TeamPlannerPyWrapper();
    ~TeamPlannerPyWrapper();
    bool formProblem(PyObject* paramFileObj, PyObject* sampleFileObj);
    bool optimize();
    PyObject* printSolution(bool flagOutput = true);
    void saveSolution(PyObject* saveFileObj, PyObject* paramFileObj);
};

#endif //TEAMPLANNER_PY_HPP