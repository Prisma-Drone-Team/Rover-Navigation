#include <iostream>
#include <chrono>

/*
 *  seed::time is a wrapper for chrono, the most precise time-counter
 *      as far as I know.
 */

namespace new_time {

//a point in the time
typedef std::chrono::high_resolution_clock::time_point t_point;

//elapsed seconds between two time points
inline double elapsed(t_point t0, t_point t1){
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    return time_span.count();
}

//get the current time point (like now() function)
inline t_point now(){
    return std::chrono::high_resolution_clock::now();
}

//stopwatch matlab-like implementing tic and toc
class Clock {
public:
    Clock(){
        t0 = new_time::now();
    }

    t_point tic(){
        t0 = new_time::now();
        return t0;
    }

    //returns elapsed time in seconds
    double toc(){
        t_point t1 = new_time::now();
        return new_time::elapsed(t0, t1);
    }

private:
    t_point t0;
};

//associate the time to a value of a generic type (template)
//  the time is set iff the value is set!
template<class T>
class TimedValue {
public:
    TimedValue(){
        time = new_time::now();
        value = T();
    }
    TimedValue(T val){
        time = new_time::now();
        value = val;
    }
    TimedValue(T val, t_point t){
        time = t;
        value = val;
    }
    void set(T val){
        time = new_time::now();
        value = val;
    }
    T get(){
        return value;
    }
    t_point getTime(){
        return time;
    }
protected:
    t_point time;
    T value;
};

}

