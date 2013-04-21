#include "NXCDefs.h"
#include "HTSMUX-driver.h"

#define Schwarzwert2 45
#define Schwarzwert3 50
#define Silberwert2 65
#define Silberwert3 65				//Werte überprüfen!!
#define Dosenwert 45
#define ramp 60
#define stime 2000
#define LongTime 5000
#define AveragePodVal 33
#define blackconst 1

//Variablen
int tempval;
bool finrampe = false;
int Speednorm = 70;
int PodPosition;
int tempposition = 0;
long intzeroval;
int GreenAttempts;
bool DownFloor;
byte count;
long sensorTime;
byte SpeedLeft;
byte SpeedRight;
int time;
bool DirectionRight;
int oben = false;
int x, y, z;
long SLTime;
long LLTime;
bool Dose;
bool Rampe = false;
byte Ultrasonicvalue;
bool Fail;
int blackiterations;
bool isInTurn;
int LeftCounter = 0;
int RightCounter = 0;

//Funktionen
void InitSensors()
{
     SetSensorLight(IN_3);
     SetSensorLight(IN_2);
     SetSensorLowspeed(IN_1);
     SetSensorColorFull(IN_4);
	 //
	 //SetSensorLowspeed(IN_4);
     //SetSensor(S4, SENSOR_TOUCH);
     //Sensor Testen...
     if (!HTSMUXscanPorts(S1))
     {
        // Scan failed, handle the error
        TextOut(0, LCD_LINE1, "Scan failed!");
        Wait(1000);
     }
     smuxSetSensorLegoLight(msensor_S1_3, true);
	 smuxSetSensorLegoLight(msensor_S1_1, true);
}
void StartTiming()
{
     intzeroval = CurrentTick();
}
void StartSL()
{
     SLTime = CurrentTick();
}
long ReturnSLTime()
{
      return (CurrentTick() - SLTime);
}
void StartLL()
{
	LLTime = CurrentTick();
}
long ReturnLLTime()
{
	return (CurrentTick() - LLTime);
}
void HandleCounting()
{
     count++;
     if(count >= 30)
     {
         count = 0;
     }
}
bool isAbleToCheck()
{
     TextOut(0, LCD_LINE2, " ");
     NumOut(0, LCD_LINE2, count);
     if(count == 0)
     {
          return true;
     }
     return false;
}
bool ReadFrontTouch()
{
     if(isAbleToCheck())
     {
          return smuxReadSensorLegoTouch(msensor_S1_4);
     }
     return false;
}
void DoRotations(long degrees, char pwr, int iterations, int rl)
{
     for(int i = 0; i < iterations; i++)
     {
         RotateMotorEx(OUT_BC, -pwr, degrees, rl,true,true);
     }
}
void TurnRight45()
{
     DoRotations(360,100,1,74);
}
void TurnLeft45()
{
     DoRotations(400,100,1,-74);
}
void TurnRightDown45()
{
     DoRotations(250,-100,1,100);
}
void TurnLeftDown45()
{
     DoRotations(270,-100,1,-100);
}

void AvoidCollision()
{
      DoRotations(300,40,1,-2);
	  TurnLeftDown45();
	  DoRotations(650, -40, 1,2);
	  TurnRightDown45();
	  DoRotations(800, -40, 1,2);
	  //TurnRightDown45();
	  DoRotations(280,-100,1,100);
	  //TurnRightDown45();
      while(SENSOR_2 > Schwarzwert2)
	  {
		  OnFwd(OUT_BC, 40);
	  }
	  DoRotations(100, -40, 1,2);
	  Off(OUT_BC);
}
bool ReadAccel(int &x, int &y, int &z)
{
     if(isAbleToCheck())
     {
          smuxReadSensorHTAccel(msensor_S1_2, x, y, z);
          return true;
     }
     return false;
}
void driveRamp()
{
     if(x > ramp)
         {
              Rampe=true;
			  Speednorm = 100;
			  finrampe=true;
		 }
	 else if(x < -ramp)
	 {
			Rampe=true;
			  Speednorm = 20;
	 }
	 else
		 {
			  Rampe = false;
			  Speednorm = 70;
		 }
}
void StartGreenLine()
{
     sensorTime = CurrentTick();
}
void CheckForLongSync()
{
     if(ReadFrontTouch())
     {
         AvoidCollision();
     }

     if(ReadAccel(x, y, z))
     {
		 NumOut(0,LCD_LINE4,Speednorm);
		 NumOut(0,LCD_LINE5,x);
		 driveRamp();
     }
     int color = SENSOR_4;
     switch(color)
     {
         case 3:
              PlayTone(5000, 20);

              GreenAttempts += 1;
              if(GreenAttempts > 12)
              {
                  StartGreenLine();
              }

         break;
         default:
              GreenAttempts = 0;
         break;
     }
     /*if((SENSOR_2 > Silberwert2) || (SENSOR_3 > Silberwert3) && finrampe == true)
     {
         DownFloor = false;
     }*/
}
long ReturnTime()
{
     return (CurrentTick() - intzeroval);
}
void TurnRight()
{
	 isInTurn = true;
     /*if(Rampe)
	 {
		SpeedLeft = 0;
		SpeedRight = 100;
	 }
	 else
	 {*/
		SpeedLeft = -100;
		SpeedRight = 100;
	// }
	 DirectionRight = true;
     StartTiming();
}
void TurnLeft()
{
	 isInTurn = true;
     /*if(Rampe)
	 {
		TextOut(0,LCD_LINE2,"GOOOOOODCODE");
		SpeedLeft = 50;
		SpeedRight = 0;
	 }
	 else
	 {*/
		SpeedLeft = 100;
		SpeedRight = -100;
	 //}
     DirectionRight = false;
     StartTiming();
}
long ReturnSensorTime()
{
     return (CurrentTick() - sensorTime);
}
void Align()
{
     //an Alufolie ausrichten
     if(SENSOR_3 > Silberwert3)
          {
              while((SENSOR_3 < Silberwert3) || (SENSOR_2 < Silberwert2))
              {
                  if(SENSOR_3 < Silberwert3)
                  {
                      OnFwd(OUT_C, -30);
                  }
                  if(SENSOR_2 < Silberwert2)
                  {
                      OnFwd(OUT_B, 30);
                  }
				  Wait(1);
				  Off(OUT_BC);
              }
			  OnFwd(OUT_C,30);
			  Wait(300);
          }
         if(SENSOR_2 > Silberwert2)
          {
              while((SENSOR_3 < Silberwert3) || (SENSOR_2 < Silberwert2))
              {
                  if(SENSOR_2 < Silberwert2)
                  {
                      OnFwd(OUT_B, -30);
                  }
                  if(SENSOR_3 < Silberwert3)
                  {
                      OnFwd(OUT_C, 30);
                  }
				  Wait(1);
				  Off(OUT_BC);
              }
			  OnFwd(OUT_B,30);
			  Wait(300);
          }
}
int ReadBackLight()
{
     return smuxSensorLegoLightNorm(msensor_S1_3);
}
int ReadSideLight()
{

          return smuxSensorLegoLightNorm(msensor_S1_1);

}
void ReadUltrasonic()
{
    Ultrasonicvalue = SensorUS(IN_4);
}
bool CheckForDose()
{
     if(!Dose)
     {
              RotateMotor(OUT_A, -100, 180);
			  //RotateMotor(OUT_A, 80, 190);
			  //OnFwd(OUT_BC, 50);
			  //Wait(5);
              NumOut(0, LCD_LINE3, ReadBackLight());
              if(ReadBackLight() > Dosenwert)
              {
                     OnFwd(OUT_A, -100);
                     Wait(1000);
                     return true;
              }
              RotateMotor(OUT_A, 100, 180);
              return false;
     }
     return true;
}
void MicroLine()
{
    DoRotations(360,100,1,2);
}
void StartUpper()
{
	DoRotations(1200,-40,1,10);
}
void MinusMicroLine()
{
	DoRotations(360,-100,1,-2);
}
void PutDownCan()
{
	  OnFwdReg(OUT_BC,100,OUT_REGMODE_SYNC);
	  Wait(600);
	  TurnLeft45();
	  TurnLeft45();
	  Off(OUT_BC);
	  OnRevReg(OUT_BC,30,OUT_REGMODE_SYNC);
	  Wait(1000);
	  Off(OUT_BC);
	  OnFwd(OUT_A,30);
	  Wait(1000);
	  Off(OUT_A);
	  Wait(5000);
	  OnRevReg(OUT_BC,30,OUT_REGMODE_SYNC);
	  Wait(1000);
}
void LongLine1()
{
	//DoRotations(2160, 100, 1,2);
	StartLL();
	while(ReturnLLTime() < LongTime)											//LongTime abstimmen!!
	{	
		OnRev(OUT_B,60);
		OnRev(OUT_C,57);
		ReadUltrasonic();
		NumOut(0,LCD_LINE6,Ultrasonicvalue);
		if((Ultrasonicvalue <= 35) && (Ultrasonicvalue != 0))
		{
			Off(OUT_ABC);
			OnFwd(OUT_BC,50);
			Wait(2500);
			if(Ultrasonicvalue <= 10)
			{
				OnFwd(OUT_C,50);
				OnRev(OUT_B,40);
				Wait(300);
				Off(OUT_BC);
			}
			if((Ultrasonicvalue > 10) && (Ultrasonicvalue <= 20))
			{
				OnFwd(OUT_C,50);
				OnRev(OUT_B,40);
				Wait(500);
				Off(OUT_BC);
			}
			if(Ultrasonicvalue > 20)
			{
				OnFwd(OUT_C,50);
				OnRev(OUT_B,40);
				Wait(700);
				Off(OUT_BC);
			}
			int i = 0;
			while(!Dose)
			{
				MicroLine();
				Dose = CheckForDose();
				i++;
			}
			for(int b = 0;b<i;b++)
			{
				MinusMicroLine();
			}
			OnRev(OUT_BC,50);
			Wait(3500);
			Off(OUT_BC);
		}
		if(!Dose)
		{
			Fail = true;
		}
	}
}
void LongLine2()
{
     DoRotations(1800, 100, 1,2);
}
long MiniLine()
{
     int counter = 0;
     long averagevalue = 0;
     StartSL();
     while(ReturnSLTime() < 1200)
     {
          NumOut(0,LCD_LINE5,ReturnSLTime());
          OnRevReg(OUT_BC,100,OUT_REGMODE_SYNC);
          counter++;
          averagevalue += ReadSideLight();
     }
     averagevalue = averagevalue/counter;
     Off(OUT_BC);
     return averagevalue;
}
