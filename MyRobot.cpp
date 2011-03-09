#include "WPILib.h"

/**
 * Stuff that I am thinking, and how to get it done:
 *  All of the drive motors still need to get encoders on them
 * 		Grace from electrical wired up the system for the encoders, but have not been all tested, she said that she made them so the metel part would be facing outward.
 * 		You will have to check the number of ticks on this, I am thinking about 240 or something, also if I remember correctly, you will have to set the PID for the Jaguar, (mostly should just require P and maybe a little D)
 * The encoders on the arm still need to be wired up (400 ticks per rotation)
 * 	The encoder at the shoulder should be wired to the cRIO and not the Jaguar
 * 		The reason for this is so that we can use a more complex control system that was suggested by Don
 * 		FYI: Don suggested using a double PID loop to control the arm, there are some complications with that which I (Matthew) have spent a lot of time figuring out how to get working.
 * 		 		So if you are stuck with getting this working, them just try a single PID loop, I might try to add some sample code that you can edit to get working, idk...	
 * I have made both of the plugs for the wires out on the arm and they are in the encoder box, they just need to be wired to the ends of the cable, The sheet with the information about how to do this is contained in the box as well
 * The encoder at the elbow should be wired to the Jaguar, we are going to just run the internal Pid loop provided by the Jaguar on this.
 * 
 * The compressor is wired up and programed to work, but both of the valves need to be wired
 * 		With the valves, each side comtains a diode and thus if you wire one of them backwards, then they can both be controled by the same skipe (what we are going to do)
 * 
 * If you have any problems or question about how I was going to be doing something, then let me know: (310) 483-3831 (if you did not have it all ready).
 */

/**
 * This current code is as it was when the robot was running, there is the line tracking code, but noting else using sensors
 */


#include <iostream>
using namespace std;

#define debug(x) \
	std::cerr << __LINE__ << ": " << x << std::endl;

class lowPass {
private:
	float lastValue;
	float change;
	float last;
public:
	lowPass(float c):lastValue(0), change(c) {}
	float operator () (float value, CANJaguar* jag) {
		lastValue = lastValue + (value - lastValue) * change;
		if(jag) jag->Set(lastValue);
		return lastValue;
	}
	float operator () (float value) {
		return lastValue = lastValue + (value - lastValue) * change;
	}
	float geto (float value) {
		return lastValue = lastValue + (value - lastValue) * (abs(lastValue) - abs(value) > 0 ? .9 : change);
	}
};

class SimplePID {
private:
	double last_error;
	double integral;
	double P,I,D;
	//char count;
public:
	SimplePID (double p, double i, double d): last_error(0), integral(0) ,P(p), I(i), D(d) {}
	float operator () (double target, double at) {
		double error = target - at;
		integral+= error;
		double derivative = error - last_error;
		last_error = error;
//		if(count++%30==0) cerr << error << '\t' << integral;
		return P*error + I*integral + D*derivative;
	}
};


class RobotSystem : public SimpleRobot
{
	//RobotDrive myRobot; // robot drive system
	bool robotInted;
	
	Joystick stick; // only joystick
	Joystick stick2;
	CANJaguar *Dlf, *Dlb, *Drf, *Drb, *arm1, *arm2;
	DigitalInput line1, line2, line3;
	Task updateCAN, cameraTask;
	Compressor compressor;
	Encoder EncArm, EncClaw;
	SimplePID PIDArm, PIDClaw;
	lowPass LowArm;
	Relay MiniBot1, MiniBot2, ClawGrip;
	DigitalInput LimitClaw;
	
	float ShoulderArmCurrent;
public:
	RobotSystem(void):
		robotInted(false)
		,stick(1)		// as they are declared above.
		,stick2(2)
		,line1(10)
		,line2(11)
		,line3(12)
		//,camera(AxisCamera::GetInstance())
		,updateCAN("CANUpdate",(FUNCPTR)UpdateCAN)
		,cameraTask("CAMERA", (FUNCPTR)CameraTask)
		,compressor(14,1)
		,EncArm(2,3)
		,EncClaw(5,6)
		,PIDArm(.04,0000024,0)
		,PIDClaw(.014,.0000014,0)
		,LowArm(.1)
		,MiniBot1(4)
		,MiniBot2(2)
		,ClawGrip(3)
		,LimitClaw(7)
	{
	//	myRobot.SetExpiration(0.1);
		GetWatchdog().SetEnabled(false);
		GetWatchdog().SetExpiration(1);
		compressor.Start();
		debug("Waiting to init CAN");
		Wait(2);
		
		Dlf = new CANJaguar(6,CANJaguar::kSpeed);
		Dlb = new CANJaguar(3,CANJaguar::kSpeed);
		Drf = new CANJaguar(7,CANJaguar::kSpeed);
		Drb = new CANJaguar(2,CANJaguar::kSpeed);
		arm1 = new CANJaguar(5);
		arm2 = new CANJaguar(4);
		
		
		EncArm.SetDistancePerPulse(.00025);
		EncClaw.SetDistancePerPulse(.00025);
		EncClaw.SetReverseDirection(false);
		EncArm.Reset();
		EncClaw.Reset();
		
		
		updateCAN.Start((int)this);
		cameraTask.Start((int)this);
		EncArm.Start();
		EncClaw.Start();
		debug("done initing");
	}
	
	~RobotSystem() {
		debug("Deleting robot");
		updateCAN.Stop();
		cameraTask.Stop();
	}
	
	static int UpdateCAN (RobotSystem *self) {
		char count=0;
		// runs at 28.5Hz
		double lastTime=0;
		while(true) {
			Wait(.035 - (GetClock() - lastTime));
			lastTime = GetClock();
			if(self->IsEnabled())
				CANJaguar::UpdateSyncGroup(2);
			if(count++%10==0) {
				self->ShoulderArmCurrent = self->arm1->GetOutputCurrent();
			}
		}
		return 0;
	}
	
	static int CameraTask (RobotSystem *self) {
		//AxisCamera &camera = AxisCamera::GetInstance();
		while(true) {
			Wait(.08);
			//if(camera.IsFreshImage()) {}
		}
		return 0;
	}
	

	float range (float f) {
		if(f > 1.0)
			return 1.0;
		if(f < -1.0)
			return -1.0;
		if(f < .1 && f > -.1)
			return 0;
		return f;
	}
	
	void RobotInit(void) {
		debug("Robot init");
		//AxisCamera &camera = AxisCamera::GetInstance();
		//endTask();
		//camera.WriteResolution(AxisCamera::kResolution_320x240);
		//camera.WriteCompression(20);
		//camera.WriteBrightness(0);
		//camera.WriteMaxFPS(15);
	}
	
	void initRobot () {
			cerr << "running init\n";
			Dlf->EnableControl(0);
			Dlb->EnableControl(0);
			Drf->EnableControl(0);
			Drb->EnableControl(0);
			arm1->EnableControl();
			arm2->EnableControl();
			
			Dlf->ConfigEncoderCodesPerRev(250);
			Dlf->SetPID(1,0,0);
			Dlb->ConfigEncoderCodesPerRev(250);
			Dlb->SetPID(1,0,0);
			Drf->ConfigEncoderCodesPerRev(250);
			Drf->SetPID(1,0,0);
			Drb->ConfigEncoderCodesPerRev(250);
			Drb->SetPID(1,0,0);
			Wait(.1);
			if(robotInted==false) {
				int count=170;
				arm2->Set(-.3);
				while(count-->0 && LimitClaw.Get() == 1) Wait(.005);
				arm2->Set(.15);
				while(count-->0 && LimitClaw.Get() == 0) Wait(.005);
				arm2->Set(0);
				EncClaw.Reset();
				
				robotInted = true;
			}
		}
	
	void Disabled (void) {
		debug("disable");			
		Dlf->StopMotor();
		Dlb->StopMotor();
		Drf->StopMotor();
		Drb->StopMotor();
	}
	
	
	void Arm(double joy) {
		double speed = PIDArm(0, EncArm.Get());
		//if(speed > .5) speed = .5;
		if(speed < -.5) speed = -.5;
		arm1->Set(LowArm(speed),2);
	}
	
	void Claw(double &joy) {
		if(joy < 10) joy = 10;
		if(joy > 180) joy = 180;
		int location = EncClaw.Get();
		double speed = PIDClaw(joy, location);
		//if(location < 15) speed *= .2;
		if(speed > .32) speed = .32;
		if(speed < -.32) speed = -.32;
		arm2->Set(speed,2);
	}
	
	void Drive (float speed, float turn, float strafe) {
		Dlf->Set(range(speed + turn + strafe)*250, 2);
		Dlb->Set(range(speed + turn - strafe)*250, 2);
		Drf->Set(range(-speed + turn + strafe)*250, 2);
		Drb->Set(range(-speed + turn - strafe)*250, 2);
		//CANJaguar::UpdateSyncGroup(2);
					
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		//myRobot.SetSafetyEnabled(false);
		//myRobot.Drive(0.5, 0.0); 	// drive forwards half speed
		//Wait(2.0); 				//    for 2 seconds
	//	myRobot.Drive(0.0, 0.0); 	// stop robot
		//AxisCamera &camera = AxisCamera::GetInstance();
		//camera.IsFreshImage();
		initRobot();
		debug("in auto");
		Wait(1.0);
		GetWatchdog().SetEnabled(false);
		int count=0;
		while(IsAutonomous() && count < 100) {
			count++;
			bool l1 = line1.Get();
			bool l2 = line2.Get();
			bool l3 = line3.Get();
			if(l1 && l2) {
				Drive(.25, -.25, 0);
			}else if(l3 && l2) {
				Drive(.25,.25, 0);
			}else if(l3 && l1) {
				Drive(.2, .3, 0);
			}else if(l2) {
				Drive(.45,0,0);
			}else if(l1) {
				Drive(0,0,.4);
			}else if(l3) {
				Drive(0,0,-.4);
			}else{
				Wait(.005);
				Drive(0,0,0);
			}
			Wait(0.05);
		}
		
		//Wait(1.0);
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	
	void OperatorControl(void)
	{

		initRobot();
		
		debug("in telop");
		compressor.Start();
		GetWatchdog().SetEnabled(true);
		/*int l1, l2, l3;
		while (IsOperatorControl()) {
			GetWatchdog().Feed();
			//char val = (line1.Get() & 0x01) | (line2.Get() & 0x02) | (line3.Get() & 0x04);
			//if(l1 != line1.Get() || l2 != line2.Get() || l3 != line3.Get()) {
			//	cerr << "change " << (l1 = line1.Get()) << "\t" << (l2 = line2.Get()) << "\t" << (l3 = line3.Get()) << endl;
			//}
			cerr << "Change "<< line1.Get() <<"\t" << line2.Get() << "\t" << line3.Get() << endl;
			Wait(0.2);
		}*/
		char count=0;
		// was .125 when loop at .025
		lowPass lowSpeed(.04), lowStrafe(.04), lowTurn(.04), lowClaw(.04);
		
		double ClawLocation=0;
		
		while (IsOperatorControl() && !IsDisabled())
		{
			GetWatchdog().Feed();
			float speed = -1*stick.GetRawAxis(2);
			float strafe = stick.GetRawAxis(1);
			float turn = stick.GetRawAxis(3);

			if(!stick.GetRawButton(7)) {
				speed /= 2;
				strafe /= 2;
				turn /= 2;
			}
			if(stick.GetRawButton(8)) {
				speed /= 2;
				strafe /= 2;
				turn /= 2;
			}
			
			Drive(lowSpeed(speed), lowTurn(turn), lowStrafe(strafe));		
			
			
			
			
			if(stick2.GetRawButton(10)) {
				robotInted = false;
				initRobot();
			}
			
			if(stick2.GetRawButton(5)) { // depoly out
				MiniBot1.Set(Relay::kReverse);
				MiniBot2.Set(Relay::kReverse);
			}
			if(stick2.GetRawButton(7)) { // deploy in
				MiniBot1.Set(Relay::kForward);
				MiniBot2.Set(Relay::kForward);
			}
			
			if(stick2.GetRawButton(1)) {
				ClawLocation = 156; // the "down" location
			}
			if(stick2.GetRawButton(2)) {
				ClawLocation = 56; // the 90angle
			}
			if(stick2.GetRawButton(4)) {
				ClawLocation = 0; // back
			}
			
			if(stick2.GetRawButton(6)) {
				ClawGrip.Set(Relay::kForward);
			}
			if(stick2.GetRawButton(8)) {
				ClawGrip.Set(Relay::kReverse);
			}
			
			ClawLocation += lowClaw(stick2.GetRawAxis(4)); // the right joy stick y
			Claw(ClawLocation);
			//arm2->Set(range(stick2.GetRawAxis(2)));
			
			//arm1->Set(range(stick2.GetRawAxis(4)));
			
			//Arm(stick2.GetRawAxis(4));
						
			
			if(count++%20==0){
				cerr << arm1->GetOutputCurrent() << '\t' << arm2->GetOutputCurrent() << '\t'
				 	<< EncClaw.Get() << '\t' << arm2->Get() << '\t' << LimitClaw.Get() << '\t' << ClawLocation << endl;
				//cerr << '\t' << EncArm.Get() <<'\t' << arm1->Get() << endl;
			//	cerr << Dlf->Get() << '\t' << Dlf->GetSpeed() << '\t' << Dlb->GetSpeed() <<'\t' << Drf->GetSpeed() <<'\t' << Drb->GetSpeed() <<endl;//'\t' << line1.Get() << "\t" << line2.Get() << "\t" << line3.Get() << endl;
			//	cerr << '\t' << Dlb->GetSpeed() << '\t';
			}
			
			
			Wait(0.01);				// wait for a motor update time
		}
	}
};


class PnumaticArmTest : public SimpleRobot {
private:
	Joystick stick;
	Solenoid up, down;
	Encoder enc;
	SimplePID pid;
	lowPass low;
public:
	PnumaticArmTest (void):
	stick(1)
	,up(7,7)
	,down(7,8)
	,enc(1,2)
	,pid(.045,.00000,.16)
	,low(.05)
	{
		enc.SetReverseDirection(true);
		enc.Start();
	}
	void OperatorControl(void) {
		char count=0;
		double target = 0, speed = 0;
		while(!IsDisabled()) {
			double tmpStick = -1*stick.GetRawAxis(2);
			if(tmpStick < .2 && tmpStick > -.2) tmpStick=0;
			target += tmpStick*1.5;
			int location = enc.GetRaw();
			if(stick.GetRawButton(5)) {
				up.Set(true);
				down.Set(false);
			}else if(stick.GetRawButton(7) && location > 0) {
				down.Set(true);
				up.Set(false);
			}else if(stick.GetRawButton(8)) {
				down.Set(true);
				up.Set(false);
			}else if(stick.GetRawButton(9)){
			
				speed = pid(target, location);
				if(speed > 1) {
					up.Set(true);
					down.Set(false);
				}else if(speed < -1) {
					up.Set(false);
					down.Set(true);
				}else{
					up.Set(false);
					down.Set(false);
				}
			}else if(stick.GetRawButton(10)) {
				enc.Reset();
			}else{
				up.Set(false);
				down.Set(false);
			}
			if(stick.GetRawButton(1))
				target = 2;
			if(stick.GetRawButton(4))
				target = 400;
			if(stick.GetRawButton(3))
				target = 200;
			if(stick.GetRawButton(2))
				target = 70;
				
			Wait(.02);
			while(count++%30==0) cerr << location << '\t' << target << '\t' << speed << endl;
		}
	}
};

//START_ROBOT_CLASS(RobotSystem);
START_ROBOT_CLASS(PnumaticArmTest);

extern "C" INT32 FRC_ROBOT_START () {
	return FRC_UserProgram_StartupLibraryInit();
}

