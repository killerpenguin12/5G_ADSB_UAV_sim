#pragma once
#include <pyhelper.h>
#include <vector>
#include <iostream>
#include <stdexcept>
//#include <../lib/pybind11/include/pybind11/embed.h>
//#include <pybind11/pybind11.h>

//namespace py = pybind11;

namespace uat
{

class UATModel
{

public:
    UATModel();
    ~UATModel();

    void print();
    void getConst(int numVehicles,double tf,double seed);
    void init_vehicles();
    std::vector<int> propagate(long vehicle);
    void incrementTime(int index);
    void updateVehicle(long vehicle);
    void updatePosition(long vehicleNum, std::vector<double> coord, int time);
    void printResults(long int numCollision, long int cRadius, long int maxSpeed);
    std::vector<int> listTupleToVector_Int(PyObject* incoming);
    std::vector<int> collisionTest();
    //int collisionTest();

private:
    CPyInstance pyInstance;
    CPyObject UAT_Model;
    CPyObject PyClass;
};

}