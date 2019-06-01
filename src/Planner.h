#ifndef PLANNER_H
#define PLANNER_H

#include<iostream>
#include<map>
#include<string>
#include<vector>

using std::vector;
using std::string;
using std::map;

class Vehicle{
    public:
    //constructor
    Vehicle();
    Vehicle(int lane, float s, float d, float v, float a, string state='CS');
    //destructor
    virtual ~Vehicle();

    //functions for behavior planner
    //construct functions for FSM
    










}




















#endif