#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <mutex>
#include <deque>
#include <condition_variable>
#include "TrafficObject.h"

// forward declarations to avoid include cycle
class Vehicle;



// Send should take an rvalue reference of type TrafficLightPhase whereas receive should return this type. 

template <class T>
class MessageQueue
{
public:
    T receive();
    void send(T &&);
private:
    std::deque<T> _queue;
    std::condition_variable _cond;
    std::mutex _mutex;
};

enum class TrafficLightPhase {red, green};

class TrafficLight : public TrafficObject
{
public:
    // constructor / desctructor
    TrafficLight();
    // getters / setters
    TrafficLightPhase getCurrentPhase();
    // typical behaviour methods
    void waitForGreen();
    void simulate();
    

private:
    // typical behaviour methods
    void cycleThroughPhases();
    TrafficLightPhase _currentPhase;

    std::condition_variable _condition;
    std::mutex _mutex;
    std::shared_ptr<MessageQueue<TrafficLightPhase>> _phaseQueue;
};

#endif