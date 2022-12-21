#ifndef PID_H_
#define PID_H_


typedef enum
	{
	Anti_windup_disabled=0,
	Anti_windup_enabled
	}Anti_windup_t;


typedef struct
	{
	float Kp;
	float Ki;
	float Kd;
	float Ts;
	float Set_point;
	float Anti_windup_error;
	float Outmin;
	float Outmax;
	int Anti_windup;

	}PID_Param_t;

typedef struct {
  float x;
  float y;
} Point;

void PID_setup(PID_Param_t *par);
float PID_Compute(float input);
void set_SetPoint(float sp);
int8_t* RSSI(char*devices_vector, int8_t* RSSI_beacons);
Point get_position(int8_t rss1, int8_t rss2, int8_t rss3);
#endif /* PID_H_ */