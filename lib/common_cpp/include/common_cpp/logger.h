#pragma once

#include <fstream>


namespace common
{


// I swiped this class from James's salsa code...
// This class opens a file and can write data to it with any number of
// inputs because it uses variadic templates.
class Logger
{
public:
    Logger() {}

    Logger(const std::string& filename)
    {
        open(filename);
    }

    void open(const std::string& filename)
    {
        file_.open(filename);
    }

    ~Logger()
    {
        file_.close();
    }

    template <typename... T>
    void log(T... data)
    {
        int dummy[sizeof...(data)] = { (file_.write((char*)&data, sizeof(T)), 1)... };
    }

    template <typename... T>
    void logMatrix(T... data)
    {
        int dummy[sizeof...(data)] = { (file_.write((char*)data.data(), sizeof(typename T::Scalar)*data.rows()*data.cols()), 1)... };
    }

private:
    std::ofstream file_;
};


} // namespace common
