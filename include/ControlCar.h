#pragma once

#include <XPLMInstance.h>
#include <XPLMDisplay.h>
#include <XPLMGraphics.h>
#include <XPLMMenus.h>
#include <XPLMUtilities.h>
#include <XPLMPlugin.h>
#include "XPLMProcessing.h"
#include <XPLMDataAccess.h>
#include <cstdio>
#include <cstring>
#include <math.h>

class Vehicle{
public:
    // contructor
    Vehicle(){};

    // home
    float _home_x = -21508.93f;
    float _home_y = -109.40f;
    float _home_z = 30575.24f;

    float flight_local_x;
    float flight_local_y;
    float flight_local_z;
    
    bool goStraight(XPLMDrawInfo_t &infoCurrent, bool &completed);
    void setDest(XPLMDrawInfo_t &infoDest);
    void setDest(float dest_x, float dest_y, float dest_z);
    void he_pt();
    void calcRoad(XPLMDrawInfo_t &home, float heading, float s);
    void setHome(XPLMDrawInfo_t &infoHome);
    void setHome(float dest_x, float dest_y, float dest_z);
    void setHeading(XPLMDrawInfo_t &infovehicle, float heading);
    void getPositionLocal(double Lat, double Lon, double Alt, XPLMDrawInfo_t &localPosition); // get local position from world position (lat,long,alt)
    void setPositionLocal_as_Dest(double Lat, double Lon, double Alt);
    void setPositionLocal_as_Home(double Lat, double Lon, double Alt);
    void set_rate_step(float step);

private:
    // destination
    float _dest_x;
    float _dest_y;
    float _dest_z;

    // heso line
    float _u;
    float _w;
    float _v;
    float _k;
    float _k1;

    float _rate_step = 0.015;

};