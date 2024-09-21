#ifndef PID_v1_h
#define PID_v1_h
#define PID_LIBRARY_VERSION	1.2.1 //NM modified to avoid name clashes
//NM modified
class PID
{
  public:
/*
  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1
*/  
  //Parameter types for some of the functions below
  //NM added based on someone else's comment - avoids name clashes
  enum mode_t      { AUTOMATIC = 1, MANUAL  = 0 };
  enum direction_t { DIRECT    = 0, REVERSE = 1 };
  enum action_t    { P_ON_M    = 0, P_ON_E  = 1 };

  //commonly used functions **************************************************************************

// * constructor.  links the PID to the Input, Output, and
//   Setpoint.  Initial tuning parameters are also set here.
//   (overload for specifying proportional mode)
    PID(float* Input, float* Output, float* Setpoint,   
        float Kp, float Ki, float Kd, action_t POn, direction_t ControllerDirection);

// * constructor.  links the PID to the Input, Output, and
//   Setpoint.  Initial tuning parameters are also set here
    PID(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, direction_t ControllerDirection);

    void SetMode(mode_t Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(float Min, float Max); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
  void SetTunings(float Kp, float Ki, float Kd);// * While most users will set the tunings once in the 
                                                  //   constructor, this function gives the user the option
                                                  //   of changing tunings during runtime for Adaptive control
                                                  
  void SetTunings(float Kp, float Ki, float Kd, action_t POn);// * overload for specifying proportional mode
       	  

	void SetControllerDirection(direction_t Direction);// * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
                      
  void SetSampleTime(int NewSampleTime);// * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************

	float GetKp();						  // These functions query the pid for interal values.
	float GetKi();						  //  they were created mainly for the pid front-end,
	float GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();	

  //NM added function to enable Compute() to run every time
  //so external timing tick can be used rather than the internal tick
  void runAlways();

  private:

	void Initialize();
	
	float dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	float dispKi;				//   format for display purposes
	float dispKd;				//
    
	float kp;                  // * (P)roportional Tuning Parameter
  float ki;                  // * (I)ntegral Tuning Parameter
  float kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

  float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
  float *myOutput;             //   This creates a hard link between the variables and the 
  float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                               //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	float outputSum, lastInput;

	unsigned long SampleTime;
	float outMin, outMax;
	bool inAuto, pOnE;

  //NM added
  bool m_runAlways;
};

#endif

