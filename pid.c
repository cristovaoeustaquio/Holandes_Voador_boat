#include "pid.h"
#include "math.h"
#include "stdlib.h"


float Kp,Ki,Kd,Ts,Outmin,Outmax,set_point,antiwinduperror;
float P, I, D;
float lastInput, lastError;
int windup;
float error;

unsigned long lastTime;

void PID_setup(PID_Param_t *par)
{
	Kp=par->Kp;
	Ki=par->Ki;
	Kd=par->Kd;
	Ts=(par->Ts)/1000;
	set_point=par->Set_point;
	antiwinduperror=par->Anti_windup_error;
	Outmin=par->Outmin;
	Outmax=par->Outmax;
	windup=par->Anti_windup;

	if(0==par->Anti_windup_error){antiwinduperror=10;}

	Ki = Ki*Ts;
	Kd = Kd / Ts;

}



float PID_Compute(float input)
{
	float out;

	error=(set_point-input);
	I += error; //integral term calculation

	float dInput = (input - lastInput);

	out = Kp*error + Ki*I + Kd*(error-lastError); //Compute PID output

	//windup for output:
	if(out > Outmax) out = Outmax;
	else if(out < Outmin) out = Outmin;

	//Save for later:
	lastInput = input;
	lastError = error;
	return out;

}
void set_SetPoint(float sp){
	set_point = sp;
}

int8_t* RSSI(char*devices_vector, int8_t* RSSI_beacons){
	//search str in a array and return pointer
	char *PSEB1_str = strstr(devices_vector,"Amazfit"); 
	char *PSEB2_str = strstr(devices_vector,"QCY");
	char *PSEB3_str = strstr(devices_vector,"QCY");
	//convert char to int and assing to array
	if (PSEB1_str != NULL) RSSI_beacons[0] = -1*((*(PSEB1_str-3)-'0')*10 + (*(PSEB1_str-2)-'0'));
	if (PSEB2_str != NULL) RSSI_beacons[1] = -1*((*(PSEB2_str-3)-'0')*10 + (*(PSEB2_str-2)-'0'));
	if (PSEB3_str != NULL) RSSI_beacons[2] = -1*((*(PSEB3_str-3)-'0')*10 + (*(PSEB3_str-2)-'0'));
	return RSSI_beacons;
	free (PSEB1_str);
	free (PSEB2_str);
	free (PSEB3_str);
}

Point get_position(int8_t rss1, int8_t rss2, int8_t rss3) {

  int8_t P = -69; // Abstract Value, Must be measured

  int8_t N = 2;  // NI

  // getting the distance in metters
  float d1 = pow(10,((P - rss1)/(10*N)));
  float d2 = pow(10,((P - rss2)/(10*N)));
  float d3 = pow(10,((P - rss3)/(10*N)));

  // Define the 3 known points.
  const Point B1 = {-19.866733, -43.9364666};
  const Point B2 = {-19.866425, -43.964556 };
  const Point B3 = {-19.866572, -43.964556 };

  // Calculate the position of the unknown point.

  float A = (-2*B1.x+2*B2.x);
  float B = (-2*B1.y+2*B2.y);
  float C = pow(d1,2)-pow(d2,2)-pow(B1.x,2)+pow(B2.x,2)-pow(B1.y,2)+pow(B2.y,2);
  float D = (-2*B2.x+2*B3.x);
  float E = (-2*B2.y+2*B3.y);
  float F = pow(d2,2)-pow(d3,2)-pow(B2.x,2)+pow(B3.x,2)-pow(B2.y,2)+pow(B2.y,2);

  Point p = {
    .x = ((C*E) - (F*B)) / ((E*A) - (B*D)),
    .y = ((C*D) - (F*A)) / ((B*D) - (A*E))
  };
  return p;
}

