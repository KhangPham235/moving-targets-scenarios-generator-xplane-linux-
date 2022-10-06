#include "ControlCar.h"

bool Vehicle::goStraight(XPLMDrawInfo_t &infoCurrent, bool &completed){
    static float step;
	if (_rate_step == 0.00f)
	{
		completed = false;
		return false;
	}

	if (infoCurrent.heading < 270)
	{
		if ((_dest_z - infoCurrent.z) <= _rate_step && (_dest_x - infoCurrent.x) <= _rate_step)
		{
			completed = true;
			setHome(infoCurrent); // set new home
			return true;
		}
	}
	else {
		if ((infoCurrent.z - _dest_z) <= _rate_step && (infoCurrent.x - _dest_x) <= _rate_step)
		{
			completed = true;
			setHome(infoCurrent); // set new home
			return true;
		}
	}

	he_pt();
	if (_u < 0) step = -_rate_step;
	else if (_u > 0) step = _rate_step;
	else step = 0;			// can be ignored


	infoCurrent.x = infoCurrent.x + step;
	infoCurrent.z = _home_z - _k * _home_x + _k * infoCurrent.x;
	float err = _dest_z - infoCurrent.z;
	if (fabs(err) > step)
	{
		infoCurrent.y = _home_y - _k1 * _home_x + _k1 * infoCurrent.x;
	}
	
	return true;
}

void Vehicle::setDest(XPLMDrawInfo_t &infoDest)
{
	_dest_x = infoDest.x;
	_dest_y = infoDest.y;
	_dest_z = infoDest.z;
	
	he_pt();
}

void Vehicle::setDest(float dest_x, float dest_y, float dest_z){
	if ((_dest_x == dest_x) && (_dest_y == dest_y) && (_dest_z == dest_z)) {
		return;
	}
	_dest_x = dest_x;
	_dest_y = dest_y;
	_dest_z = dest_z;

	he_pt();
}

void Vehicle::he_pt(){
	// he phuong trinh duong thang
	_u = _dest_x - _home_x;
	_v = _dest_z - _home_z;
	_w = _dest_y - _home_y;
	_k = _v / _u;
	_k1 = _w / _u;
}

void Vehicle::calcRoad(XPLMDrawInfo_t &home, float heading, float s){
	he_pt();
	float phi = atan(_k);
	_dest_z = home.z + sin(phi)*s;
	_dest_x = home.x + cos(phi)*s;
}

void Vehicle::setHome(XPLMDrawInfo_t &infoHome){
	_home_x = infoHome.x;
	_home_y = infoHome.y;
	_home_z = infoHome.z;
}

void Vehicle::setHome(float home_x, float home_y, float home_z){
	_home_x = home_x;
	_home_y = home_y;
	_home_z = home_z;
}

void Vehicle::setHeading(XPLMDrawInfo_t &infovehicle, float heading){
	infovehicle.heading = heading;
}

void Vehicle::getPositionLocal(double Lat, double Lon, double Alt, XPLMDrawInfo_t &localPosition){
	double value[3];

	XPLMWorldToLocal(Lat, Lon, Alt,&value[0],&value[1],&value[2]);

	localPosition.x = (float)value[0];
	localPosition.y = (float)value[1];
	localPosition.z = (float)value[2];
}

void Vehicle::setPositionLocal_as_Dest(double Lat, double Lon, double Alt){
	double value[3];

	XPLMWorldToLocal(Lat, Lon, Alt, &value[0], &value[1], &value[2]);

	setDest((float)value[0], (float)value[1], (float)value[2]);
}

void Vehicle::setPositionLocal_as_Home(double Lat, double Lon, double Alt){
	double value[3];

	XPLMWorldToLocal(Lat, Lon, Alt, &value[0], &value[1], &value[2]);

	setHome((float)value[0], (float)value[1], (float)value[2]);
}

void Vehicle::set_rate_step(float step){
    _rate_step = step;
}
