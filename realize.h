// Created by Fedor on 16.05.2022.
#pragma once
#include <iostream>
#include <ctime>
#include <fstream>
#include <math.h>

using namespace std;
const int K = 16;
const float pi = 3.14;
const float dt = 0.01;

class TAObject {
public:
    float x, y;
    bool f;
    TAObject();
    TAObject (float x, float y);
    ~TAObject();
};

class TLA: public TAObject{
public:
    float xc, yc, r, fi, v;
    bool landing, landed;
    TLA(float x, float y, float v, float xc, float yc);
    virtual void move (float t, int a);
};

class TAircraft: public TLA{
public:
    TAircraft(float x, float y, float v, float xc, float yc);
    void move(float t, int a);
};

class THelicopter: public TLA {
public:
    THelicopter(float x, float y, float v, float xc, float yc);
    void move(float t, int a);
};

class TAirport: public TAObject {
public:
    float l;
    TLA** LA;
    TAirport(float x, float y, float l);
    void Do (float t0, float tk);
    void show(char* a, int* b, float* L1, float* L2, float* L3);
};