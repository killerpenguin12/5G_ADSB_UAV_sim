#include "uatModel.h"


namespace uat
{

UATModel::UATModel() 
{
    // CPyInstance pyInstance;
    PyRun_SimpleString( "import sys\nsys.path.append(\"/home/jonathan/Desktop/uav_sim/lib/adsb_research\")\n" );
    CPyObject myModule, dict, python_class;
    myModule = PyImport_ImportModule("UAT_Model");
    if (myModule == nullptr) {
        PyErr_Print();
        std::cerr << "Fails to import the module.\n\n";
    }
    dict = PyModule_GetDict(myModule);
    if (dict == nullptr) {
        PyErr_Print();
        std::cerr << "Fails to get the dictionary.\n";
    }
    Py_DECREF(myModule);
    python_class = PyDict_GetItemString(dict, "UAT_Model");
    PyClass = python_class;
    if (python_class == nullptr) {
        PyErr_Print();
        std::cerr << "Fails to get the Python class.\n";
    }
    Py_DECREF(dict);
    if (PyCallable_Check(python_class)) {
        UAT_Model = PyObject_CallObject(python_class, nullptr);
        Py_DECREF(python_class);
    } else {
        std::cout << "Cannot instantiate the Python class" << std::endl;
        Py_DECREF(python_class);
    }
}

UATModel::~UATModel() {}

void UATModel::print()
{
    PyObject_CallMethod(UAT_Model, "testNoArgs", NULL);
}

void UATModel::getConst(int numVehicles, double tf, double seed)
{
    CPyObject num = PyLong_FromLong(numVehicles);
    PyObject_SetAttrString(UAT_Model, "num_vehicles", num);
    CPyObject numtime = PyFloat_FromDouble(tf);
    PyObject_SetAttrString(UAT_Model, "numSeconds", numtime);
    CPyObject seedUsed = PyLong_FromLong(seed);
    PyObject_SetAttrString(UAT_Model, "seed", seedUsed);


}

void UATModel::init_vehicles()
{
    PyObject_CallMethod(UAT_Model, "init_vehicles", NULL);

}

void UATModel::incrementTime(int index)
{
    CPyObject time = PyLong_FromLong(index);
 //   std::cout << index << std::endl;
    PyErr_Print();
    if (UAT_Model == Py_None) {
        std::cout << "null" << std::endl;
    }
    PyObject_SetAttrString(UAT_Model, "timeIndex", time);
}

void UATModel::updateVehicle(long vehicle)
{
    CPyObject vehicleNum = PyLong_FromLong(vehicle);
    PyErr_Print();
    if (UAT_Model == Py_None) {
        std::cout << "null" << std::endl;
    }
    PyObject_SetAttrString(UAT_Model, "vehicleNum", vehicleNum);
}

void UATModel::updatePosition(long vehicleNum, std::vector<double> coord, int time) 
{
    incrementTime(time);
    //std::cout << "Positions in uatModel.cpp: " << coord.at(0) << " " << coord.at(1) << " " << coord.at(2) << std::endl;
    CPyObject num = PyLong_FromLong(vehicleNum);
    PyObject_SetAttrString(UAT_Model, "vehicleNum", num);
    CPyObject x = PyFloat_FromDouble(coord.at(0));
    PyObject_SetAttrString(UAT_Model, "xPos", x);
    CPyObject y = PyFloat_FromDouble(coord.at(1));
    PyObject_SetAttrString(UAT_Model, "yPos", y);
    CPyObject z = PyFloat_FromDouble(coord.at(2));
    PyObject_SetAttrString(UAT_Model, "zPos", z);
    PyObject_CallMethod(UAT_Model, "updateVehiclePosition", NULL);
    PyErr_Print();
}

std::vector<int> UATModel::propagate(long vehicle)
{
    //CPyInstance pyInstance;
    updateVehicle(vehicle);
    CPyObject list = PyObject_CallMethod(UAT_Model, "propagate", NULL);
    PyErr_Print(); //second object after propogate
    std::vector<int> cList(listTupleToVector_Int(list));
    PyErr_Print();
    return cList;
}

void UATModel::printResults(long int numCollision, long int cRadius, long int speed)
{
    // CPyInstance pyInstance;
    CPyObject x = PyLong_FromLong(numCollision);
    PyObject_SetAttrString(UAT_Model, "size", x);
    //PyObject_CallMethod(UAT_Model, "printResults", NULL);
    CPyObject y = PyLong_FromLong(cRadius);
    PyObject_SetAttrString(UAT_Model, "Radius", y);
    //PyObject_CallMethod(UAT_Model, "printResults", NULL);
    CPyObject z = PyLong_FromLong(speed);
    PyObject_SetAttrString(UAT_Model, "maxSpeed", z);
    PyObject_CallMethod(UAT_Model, "printResults", NULL);
}

std::vector<int> UATModel::collisionTest()
//int UATModel::collisionTest()
{
    CPyObject list = PyObject_CallMethod(UAT_Model, "collisionTest", NULL);
    //CPyObject pValue = PyObject_CallMethod(UAT_Model, "collisionTest", NULL);
    //std::vector<int> clist1();
    PyErr_Print();
    std::vector<int> cList(listTupleToVector_Int(list));
    return cList;
    //return pValue;
}

// PyObject -> Vector
std::vector<int> UATModel::listTupleToVector_Int(PyObject* incoming) { //fourth after propgate in order to get our vector, This comes after our pointer
	std::vector<int> data;
	if (PyTuple_Check(incoming)) {
		for(Py_ssize_t i = 0; i < PyTuple_Size(incoming); i++) {
			PyObject *value = PyTuple_GetItem(incoming, i);
			data.push_back( PyFloat_AsDouble(value) );
		}
	} else {
		if (PyList_Check(incoming)) {
			for(Py_ssize_t i = 0; i < PyList_Size(incoming); i++) {
				PyObject *value = PyList_GetItem(incoming, i);
				data.push_back( PyFloat_AsDouble(value) );
			}
		} else {
			throw std::logic_error("Passed PyObject pointer was not a list or tuple!");
		}
	}
	return data;
}

}