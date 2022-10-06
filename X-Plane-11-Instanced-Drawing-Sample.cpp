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

#include "ControlCar.h"
#define PI 3.14159
#define DEBUG_FLIGHT_LOCATION 0 // debug for getting flight location

// user variable
bool plugin_enable = false;
bool check = false;   // check at destination?
int wp = 0;

// trang thai wp
bool checkA = false;
bool checkB = false;
bool checkC = false;
bool checkD = false;


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
XPLMProbeInfo_t probe;
Vehicle car;
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
	car.setHome(vehicle);
	if(g_instance[0] || g_instance[1] || g_instance[2])
	{
		XPLMInstanceSetPosition(g_instance[2] ? g_instance[2] : (g_instance[1] ? g_instance[1] : g_instance[0]), &vehicle, &tire);
	}
}


// function for callback to update new address of object instance
float myFlightLookCallBack(float timeLastCall, float totalTime, int counter, void * refcon)
{
	car.set_rate_step(0.015);
	if (totalTime > 30) {
		if (plugin_enable) // vehicle will move when plugin_enable is Active 
		{
			vehicle.structSize = sizeof(vehicle);

			// run rectangle
			{
				switch (wp)
				{
				case 0:  // at wp 0
					car.setHeading(vehicle, 220.0f);
					//setDest(-20187.20f, vehicle.y, 30989.07f);
					car.setPositionLocal_as_Dest(21.0390728, 105.4890007, 25.2);
					car.goStraight(vehicle, checkA);
					if (checkA)
					{
						wp = 1;
						checkA = false; // reset value
					}
					break;
				case 1: // at wp 1
					car.setHeading(vehicle, 40.0f);
					//setDest(-20123.54f, vehicle.y, 30763.81f);
					car.setPositionLocal_as_Dest(21.0418841, 105.4915273, 29);
					car.goStraight(vehicle, checkB);
					if (checkB) {
						wp = 2;
						checkB = false;
					}
					break;
				case 2:
					car.setHeading(vehicle, 220.0f);
					//setDest(-20123.54f, vehicle.y, 30763.81f);
					car.setPositionLocal_as_Dest(21.0390728, 105.4890007, 25.2);
					car.goStraight(vehicle, checkC);
					if (checkC) {
						wp = 1;
						checkC = false;
					}
					break;
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


		//sprintf(MPD_Buffer[3], "heading = %0.3f", vehicle.heading);
		//sprintf(MPD_Buffer[4], "k = %0.3f", _k);
		//sprintf(MPD_Buffer[5], "roll = %0.3f", vehicle.roll);

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