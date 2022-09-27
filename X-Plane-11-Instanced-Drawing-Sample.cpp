// Downloaded from https://developer.x-plane.com/code-sample/instanced-drawing-sample/

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

#include "test.h"

#define PI 3.14159
#define DEBUG_FLIGHT_LOCATION 0 // debug for getting flight location

// private function
bool goStraight(XPLMDrawInfo_t &infoCurrent, float step, bool &completed);
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
// user variable
bool plugin_enable = false;
bool check = false;   // check at destination?
int wp = 0;

// trang thai wp
bool checkA = false;
bool checkB = false;
bool checkC = false;
bool checkD = false;


// home
float _home_x = -21508.93f;
float _home_y = -109.40f;
float _home_z = 30575.24f;

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

static XPLMDataRef x = NULL;
static XPLMDataRef y = NULL;
static XPLMDataRef z = NULL;
static XPLMDataRef heading = NULL;
static XPLMDataRef pitch = NULL;
static XPLMDataRef roll = NULL;

float flight_local_x;
float flight_local_y;
float flight_local_z;


// add draw in screen

// Used to store data for display
char MPD_Buffer[6][80];

// Function prototypes

//float MotionPlatformDataLoopCB(float elapsedMe, float elapsedSim, int counter, void * refcon);

void MotionPlatformDataDrawWindowCallback(
	XPLMWindowID         inWindowID,
	void *               inRefcon);

void MotionPlatformDataHandleKeyCallback(
	XPLMWindowID         inWindowID,
	char                 inKey,
	XPLMKeyFlags         inFlags,
	char                 inVirtualKey,
	void *               inRefcon,
	int                  losingFocus);

int MotionPlatformDataHandleMouseClickCallback(
	XPLMWindowID         inWindowID,
	int                  x,
	int                  y,
	XPLMMouseStatus      inMouse,
	void *               inRefcon);

// Window ID
XPLMWindowID MPD_Window = NULL;

// end draw in screen

#if !XPLM300
	#error This plugin requires version 300 of the SDK
#endif

//const char * g_objPath = "lib/airport/vehicles/pushback/tug.obj";
const char * g_objPath = "lib/cars/police_car.obj";
//const char * g_objPath = "lib/airport/Common_Elements/Vehicles/Cargo_Truck.obj";
XPLMObjectRef g_object = NULL;
XPLMInstanceRef g_instance[3] = {NULL};

XPLMDrawInfo_t		vehicle;

static float tire;
float myFlightLookCallBack(float timeLastCall, float totalTime, int counter, void * refcon);



static void load_cb(const char * real_path, void * ref)
{
	XPLMObjectRef * dest = (XPLMObjectRef *) ref;
	if(*dest == NULL)
	{
		*dest = XPLMLoadObject(real_path);
	}
}

static void menu_cb(
					void *               inMenuRef,
					void *				inItemRef)
{
	if(!g_object)
	{
		XPLMLookupObjects(g_objPath, 0, 0, load_cb, &g_object);
	}
	if(g_object)
	{
		const char * drefs[] = { "sim/graphics/animation/ground_traffic/tire_steer_deg", NULL };
		if(!g_instance[0])
		{
			g_instance[0] = XPLMCreateInstance(g_object, drefs);
		}
		else if(!g_instance[1])
		{
			g_instance[1] = XPLMCreateInstance(g_object, drefs);
		}
		else if(!g_instance[2])
		{
			g_instance[2] = XPLMCreateInstance(g_object, drefs);
		}
	}
	
	x = XPLMFindDataRef("sim/flightmodel/position/local_x");
	y = XPLMFindDataRef("sim/flightmodel/position/local_y");
	z = XPLMFindDataRef("sim/flightmodel/position/local_z");
	heading = XPLMFindDataRef("sim/flightmodel/position/psi");
	pitch = XPLMFindDataRef("sim/flightmodel/position/theta");
	roll = XPLMFindDataRef("sim/flightmodel/position/phi");
	
	tire = add(2,3);
	tire = 0.0;
	tire += 10.0;
	if(tire > 45.0) tire -= 90.0;
	
	vehicle.structSize = sizeof(vehicle);
	vehicle.x = XPLMGetDataf(x);
	vehicle.y = XPLMGetDataf(y);
	vehicle.z = XPLMGetDataf(z);
	vehicle.pitch = XPLMGetDataf(pitch);
	vehicle.heading = XPLMGetDataf(heading);
	vehicle.roll = XPLMGetDataf(roll);

	//calcRoad(vehicle, 0.0, 1385.0);

	// set home as wp 0
	setHome(vehicle);
	if(g_instance[0] || g_instance[1] || g_instance[2])
	{
		XPLMInstanceSetPosition(g_instance[2] ? g_instance[2] : (g_instance[1] ? g_instance[1] : g_instance[0]), &vehicle, &tire);
	}
}


// function for callback to update new address of object instance
float myFlightLookCallBack(float timeLastCall, float totalTime, int counter, void * refcon)
{
	if (totalTime > 30) {
		if (plugin_enable) // vehicle will move when plugin_enable is Active 
		{
			vehicle.structSize = sizeof(vehicle);

			// run rectangle
			{
				switch (wp)
				{
				case 0:  // at wp 0
					setHeading(vehicle, 220.0f);
					//setDest(-20187.20f, vehicle.y, 30989.07f);
					setPositionLocal_as_Dest(21.0390728, 105.4890007, 25.2);
					goStraight(vehicle, 0.015, checkA);
					if (checkA)
					{
						wp = 1;
						checkA = false; // reset value
					}
					break;
				case 1: // at wp 1
					setHeading(vehicle, 40.0f);
					//setDest(-20123.54f, vehicle.y, 30763.81f);
					setPositionLocal_as_Dest(21.0418841, 105.4915273, 29);
					goStraight(vehicle, 0.015, checkB);
					if (checkB) {
						wp = 2;
						checkB = false;
					}
					break;
				case 2:
					setHeading(vehicle, 220.0f);
					//setDest(-20123.54f, vehicle.y, 30763.81f);
					setPositionLocal_as_Dest(21.0390728, 105.4890007, 25.2);
					goStraight(vehicle, 0.015, checkC);
					if (checkC) {
						wp = 1;
						checkC = false;
					}
					break;
				// 	break;
				// case 2: // at wp 2
				// 	setHeading(vehicle, 287.0f);
				// 	//setDest(-21445.3f, vehicle.y, 30350.10f);
				// 	setPositionLocal_as_Dest(21.2230614, 105.7919347, 0);
				// 	goStraight(vehicle, 0.5, checkC);
				// 	if (checkC)
				// 	{
				// 		wp = 3;
				// 		checkC = false;
				// 	}
				// 	break;
				// case 3: // at wp 3
				// 	setHeading(vehicle, 17.0f);
				// 	//setDest(-21508.93f, vehicle.y,30575.24f);
				// 	setPositionLocal_as_Dest(21.2252054, 105.7927193, 0);
				// 	goStraight(vehicle, 0.1, checkD);
				// 	if (checkD)
				// 	{
				// 		wp = 0;
				// 		checkD = false;
				// 	}
				// 	break;
				default: break;
				}
			}
			
		}

		XPLMInstanceSetPosition(g_instance[2] ? g_instance[2] : (g_instance[1] ? g_instance[1] : g_instance[0]), &vehicle, &tire);

		// for debug position 
		if (DEBUG_FLIGHT_LOCATION)
		{
			flight_local_x = XPLMGetDataf(x);
			flight_local_y = XPLMGetDataf(y);
			flight_local_z = XPLMGetDataf(z);

			sprintf(MPD_Buffer[0], "x = %0.3f", flight_local_x);
			sprintf(MPD_Buffer[1], "y = %0.3f", flight_local_y);
			sprintf(MPD_Buffer[2], "z = %0.3f", flight_local_z);
		}
		else
		{
			sprintf(MPD_Buffer[0], "x = %0.3f", vehicle.x);
			sprintf(MPD_Buffer[1], "y = %0.3f", vehicle.y);
			sprintf(MPD_Buffer[2], "z = %0.3f", vehicle.z);
		}


		sprintf(MPD_Buffer[3], "heading = %0.3f", vehicle.heading);
		sprintf(MPD_Buffer[4], "k = %0.3f", _k);
		sprintf(MPD_Buffer[5], "roll = %0.3f", vehicle.roll);

	}



	return -1;
}

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc)
{
	std::strcpy(outName, "InstancingSample");
	std::strcpy(outSig, "lr.samples.instancing");
	std::strcpy(outDesc, "Sample plugin demonstrating the instancing API");
	
	int my_slot = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "instancing_sample", NULL, 0);
	XPLMMenuID m = XPLMCreateMenu("instancing_sample", XPLMFindPluginsMenu(), my_slot, menu_cb, NULL);
	XPLMAppendMenuItem(m, "add instance", NULL, 0);
	XPLMRegisterFlightLoopCallback(
		myFlightLookCallBack,	/* Callback */
		2.0,					/* Interval */
		NULL);					/* refcon not used. */

	// add windown
	

	MPD_Window = XPLMCreateWindow(
		50, 600, 200, 500,								/* Area of the window. */
		1,												/* Start visible. */
		MotionPlatformDataDrawWindowCallback,			/* Callbacks */
		MotionPlatformDataHandleKeyCallback,
		MotionPlatformDataHandleMouseClickCallback,
		NULL);											/* Refcon - not used. */


	memset(MPD_Buffer, 0, sizeof(MPD_Buffer));

	return 1;
}

PLUGIN_API void	XPluginStop(void)
{
	for(int i = 0; i < 3; ++i)
	{
		if(g_instance[i])
			XPLMDestroyInstance(g_instance[i]);
	}
	if(g_object)
		XPLMUnloadObject(g_object);
	XPLMUnregisterFlightLoopCallback(myFlightLookCallBack, NULL);
	XPLMDestroyWindow(MPD_Window);
	//XPLMUnregisterFlightLoopCallback(MotionPlatformDataLoopCB, NULL);
}

PLUGIN_API int XPluginEnable(void)
{
	plugin_enable = true;
	return 1;
}

PLUGIN_API void XPluginDisable(void)
{
	plugin_enable = false;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, long, void*)
{
}


// add windown

//---------------------------------------------------------------------------
// Mandatory callback for SDK 2D Window
// Used to display the data to the screen

void MotionPlatformDataDrawWindowCallback(
	XPLMWindowID         inWindowID,
	void *               inRefcon)
{

	float		rgb[] = { 1.0, 1.0, 1.0 };
	int			l, t, r, b;

	XPLMGetWindowGeometry(inWindowID, &l, &t, &r, &b);
	XPLMDrawTranslucentDarkBox(l, t, r, b);

	for (int i = 0; i < 6; i++)
		XPLMDrawString(rgb, l + 10, (t - 20) - (10 * i), MPD_Buffer[i], NULL, xplmFont_Basic);
}

//---------------------------------------------------------------------------
// Mandatory callback for SDK 2D Window
// Not used in this plugin

void MotionPlatformDataHandleKeyCallback(
	XPLMWindowID         inWindowID,
	char                 inKey,
	XPLMKeyFlags         inFlags,
	char                 inVirtualKey,
	void *               inRefcon,
	int                  losingFocus)
{
}

//---------------------------------------------------------------------------
// Mandatory callback for SDK 2D Window
// Not used in this plugin

int MotionPlatformDataHandleMouseClickCallback(
	XPLMWindowID         inWindowID,
	int                  x,
	int                  y,
	XPLMMouseStatus      inMouse,
	void *               inRefcon)
{
	return 1;
}


// Tk FUNCTION
void setHome(XPLMDrawInfo_t &infoHome)
{
	_home_x = infoHome.x;
	_home_y = infoHome.y;
	_home_z = infoHome.z;
}

void setHome(float home_x, float home_y, float home_z)
{
	_home_x = home_x;
	_home_y = home_y;
	_home_z = home_z;
}

void he_pt()
{
	// he phuong trinh duong thang
	_u = _dest_x - _home_x;
	_v = _dest_z - _home_z;
	_w = _dest_y - _home_y;
	_k = _v / _u;
	_k1 = _w / _u;
}
void setDest(XPLMDrawInfo_t &infoDest)
{
	_dest_x = infoDest.x;
	_dest_y = infoDest.y;
	_dest_z = infoDest.z;
	
	he_pt();

}

void setDest(float dest_x, float dest_y, float dest_z)
{
	if ((_dest_x == dest_x) && (_dest_y == dest_y) && (_dest_z == dest_z)) {
		return;
	}
	_dest_x = dest_x;
	_dest_y = dest_y;
	_dest_z = dest_z;

	he_pt();

}

bool goStraight(XPLMDrawInfo_t &infoCurrent, float step, bool &completed)
{
	float _step;
	if (step == 0)
	{
		completed = false;
		return false;
	}

	if (infoCurrent.heading < 270)
	{
		if ((_dest_z - infoCurrent.z) <= step && (_dest_x - infoCurrent.x) <= step)
		{
			completed = true;
			setHome(infoCurrent); // set new home
			return true;
		}
	}
	else {
		if ((infoCurrent.z - _dest_z) <= step && (infoCurrent.x - _dest_x) <= step)
		{
			completed = true;
			setHome(infoCurrent); // set new home
			return true;
		}
	}


	he_pt();
	if (_u < 0) _step = -step;
	else if (_u > 0) _step = step;
	else _step = 0;			// can be ignored


	infoCurrent.x = infoCurrent.x + _step;
	infoCurrent.z = _home_z - _k * _home_x + _k * infoCurrent.x;
	float err = _dest_z - infoCurrent.z;
	if (fabs(err) > step)
	{
		infoCurrent.y = _home_y - _k1 * _home_x + _k1 * infoCurrent.x;
	}
	
	return true;
}

void calcRoad(XPLMDrawInfo_t &home, float heading, float s)
{
	he_pt();
	float phi = atan(_k);
	_dest_z = home.z + sin(phi)*s;
	_dest_x = home.x + cos(phi)*s;
}

void setHeading(XPLMDrawInfo_t &infovehicle, float heading)
{
	infovehicle.heading = heading;
}


void getPositionLocal(double Lat, double Lon, double Alt,XPLMDrawInfo_t &localPosition)
{
	double value[3];

	XPLMWorldToLocal(Lat, Lon, Alt,&value[0],&value[1],&value[2]);

	localPosition.x = (float)value[0];
	localPosition.y = (float)value[1];
	localPosition.z = (float)value[2];
}

void setPositionLocal_as_Home(double Lat, double Lon, double Alt)
{
	double value[3];

	XPLMWorldToLocal(Lat, Lon, Alt, &value[0], &value[1], &value[2]);

	setHome((float)value[0], (float)value[1], (float)value[2]);
}

void setPositionLocal_as_Dest(double Lat, double Lon, double Alt)
{
	double value[3];

	XPLMWorldToLocal(Lat, Lon, Alt, &value[0], &value[1], &value[2]);

	setDest((float)value[0], (float)value[1], (float)value[2]);
}