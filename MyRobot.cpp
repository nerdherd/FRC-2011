#include "WPILib.h"


#include <iostream>
using namespace std;

#define NDEBUG

#ifndef NDEBUG
#define debug(x) \
	std::cerr << __LINE__ << ": " << x << std::endl;
#else
#define debug(x) {}
#endif

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
		if(integral < -25) integral = -25;
		if(integral > 25) integral = 25;
		return P*error + I*integral + D*derivative;
	}
};


class RobotSystem : public SimpleRobot
{
	//RobotDrive myRobot; // robot drive system
	bool robotInted;
	
	Joystick stick; // only joystick
	Joystick stick2;
	CANJaguar *Dlf, *Dlb, *Drf, *Drb, *arm1, *arm1_sec, *arm2;
	DigitalInput line1, line2, line3;
	Task updateCAN, cameraTask;
	Compressor compressor;
	Encoder EncArm, EncClaw;
	SimplePID PIDArm, PIDClaw;
	lowPass LowArm;
	Solenoid MiniBot1a, MiniBot1b, MiniBot2a, MiniBot2b, ClawOpen, ClawClose;
	//Relay MiniBot1, MiniBot2, ClawGrip;
	DigitalInput LimitClaw, LimitArm;
	
	Timer miniBotTime;
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
		,PIDArm(.04,0,0) // .002, .033
		,PIDClaw(.014,.0000014,0)
		,LowArm(.1)
		/*
		,MiniBot1(4)
		,MiniBot2(2)
		,ClawGrip(3)
		*/
		,MiniBot1a(8,1)
		,MiniBot1b(8,2)
		,MiniBot2a(8,3)
		,MiniBot2b(8,4)
		,ClawOpen(8, 8)
		,ClawClose(8,7)
		,LimitClaw(7)
		,LimitArm(13)
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
		arm1_sec = new CANJaguar(8);
		arm2 = new CANJaguar(4);
		
		
		EncArm.SetDistancePerPulse(.00025);
		EncClaw.SetDistancePerPulse(.00025);
		EncClaw.SetReverseDirection(false);
		EncArm.SetReverseDirection(true);
		EncArm.Reset();
		EncClaw.Reset();
		
		
		updateCAN.Start((int)this);
		//cameraTask.Start((int)this);
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
			if(self->IsEnabled()) {
				CANJaguar::UpdateSyncGroup(2);
				CANJaguar::UpdateSyncGroup(3);
			}
			/*
			if(count++%10==0) {
				self->ShoulderArmCurrent = self->arm1->GetOutputCurrent();
			}
			*/
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
			arm1_sec->EnableControl();
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
				int count=220;
				arm2->Set(-.3);
				while(count-->0 && LimitClaw.Get() == 1) Wait(.005);
				arm2->Set(.15);
				while(count-->0 && LimitClaw.Get() == 0) Wait(.005);
				arm2->Set(0);
				if(count>0)
					EncClaw.Reset();
				arm1->Set(-.3);
				arm1_sec->Set(-.3);
				while(count-->0 && LimitArm.Get() == 1) Wait(.005);
				arm1->Set(.5);
				arm1_sec->Set(.5);
				while(count-->0 && LimitArm.Get() == 0) Wait(.005);
				if(count>0)
					EncArm.Reset();
				arm1->Set(0);
				arm1_sec->Set(0);
				
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
		int location = EncArm.Get();
		/*
		if(location < 10 && joy < 0) joy = 0;
		if(location > 110 && joy > 0) joy = 0;
		arm1->Set(joy);
		arm1_sec->Set(joy);
		return;
		*/
		
		if(joy < -10) joy = -10;
		if(joy > 110) joy = 110;
		
		double speed = PIDArm(joy, location);
		if(speed > .5) speed = .5;
		if(speed < -.3) speed = -.3;
		if(speed < 0 && location < 10) speed = 0;
		if(speed > 0 && location > 110) speed = 0;
		speed = LowArm(speed);
		if(speed < .01 && speed > -.01) speed = 0;
		arm1->Set(speed,3);
		arm1_sec->Set(speed,3);
		
	}
	
	void Claw(double joy) {
		if(joy < 10) joy = 10;
		if(joy > 230) joy = 230;
		int location = EncClaw.Get();
		double speed = PIDClaw(joy, location);
		//if(location < 15) speed *= .2;
		if(speed > .32) speed = .32;
		if(speed < -.32) speed = -.32;
		if(speed < .1 && speed > -.1) speed = 0;
		arm2->Set(speed,2);
	}
	
	void Drive (float speed, float turn, float strafe) {
		Dlf->Set(range(speed + turn + strafe)*250, 2);
		Dlb->Set(range(speed + turn - strafe)*250, 2);
		Drf->Set(range(-speed + turn + strafe)*250, 2);
		Drb->Set(range(-speed + turn - strafe)*250, 2);
		//CANJaguar::UpdateSyncGroup(2);
					
	}

	void Autonomous(void)
	{
		GetWatchdog().SetEnabled(false);
		AxisCamera &camera = AxisCamera::GetInstance();
		initRobot();
#ifdef NDEBUG
		//return;
#endif
		debug("in auto");
		Wait(1.0);
		int count=0;
		Wait(.5);
		Arm(0);
		Claw(0);
		/*
		Drive(.25, 0,0);
		Wait(3);
		Drive(0,0,0);
		return;*/
		Drive(.25, 0, 0);
		while(IsAutonomous() && count++ < 280 && !IsDisabled()) {
			Arm(60);
			Claw(90);
			/*
			bool l1 = line1.Get();
			bool l2 = line2.Get();
			bool l3 = line3.Get();
			cerr << l1 << '\t' << l2 << '\t' << l3 << endl;
			if(l1 && l2 && l3) {
				count=310;
				break;
			}else if(l1 && l2) {
				Drive(.1, -.25, 0);
			}else if(l3 && l2) {
				Drive(.1,.25, 0);
			}else if(l3 && l1) {
				count -= 100;
				Drive(.1, .6, 0);
			}else if(l2) {
				Drive(.25,0,0);
			}else if(l1) {
				Drive(0,0,-.3);
			}else if(l3) {
				Drive(0,0,.3);
			}else{
				//Wait(.02);
				//Drive(.1,0,0);
			}*/
			Wait(0.01);
		}
		while(IsAutonomous() && count++ < 310 && !IsDisabled()) {
			Drive(.001,0,0);
			Arm(60);
			Claw(95);
			Wait(.01);
		}
		ClawOpen.Set(true);
		ClawClose.Set(false);
		while(IsAutonomous() && count++ < 330 && !IsDisabled()) {
			Arm(55);
			Claw(95);
			Drive(0,0,0);
			Wait(.01);
		}
		while(IsAutonomous() && count++ < 600 && !IsDisabled()) {
			Arm(50);
			Claw(95);
			Drive(-.1,0,0);
			Wait(.01);
		}
		Wait(.1);
		ClawOpen.Set(false);
		ClawClose.Set(true);
		while(IsAutonomous() && count++ < 800 && !IsDisabled()) {
			Arm(0);
			Claw(0);
			Drive(-.1,0,0);
			Wait(.01);
		}
		while(IsAutonomous() && !IsDisabled()) {
			Drive(0,0,0);
			Claw(0);
			Arm(0);
		}
		
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	
	void OperatorControl(void)
	{
		AxisCamera &camera = AxisCamera::GetInstance();
		miniBotTime.Start();
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
		char count=0, pneumaticCount=0;
		// was .125 when loop at .025
		lowPass lowSpeed(.04), lowStrafe(.04), lowTurn(.04), lowClaw(.04), lowArm(.04), lowArmLoc(.05);
		
		double ClawLocation=0, ArmLocation=0, OldArmLocation=0;
		
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
			if(stick.GetRawButton(2)) {
				speed = 0;
				turn = 0;
			}
			
			Drive(lowSpeed(speed), lowTurn(turn), lowStrafe(strafe));		
			
			
			
#ifndef NDEBUG
			if(stick2.GetRawButton(10)) {
				robotInted = false;
				initRobot();
			}
#endif
			
			
			if(stick2.GetRawButton(7) && (miniBotTime.Get() >= 110 || (stick2.GetRawButton(9) && stick2.GetRawButton(10)))) { // launcher
				// the quick launcher
				MiniBot1a.Set(true);
				MiniBot1b.Set(false);
				
			} 
			if(!stick2.GetRawButton(10) && stick2.GetRawButton(9)) { // deploy in
				MiniBot2a.Set(false);
				MiniBot2b.Set(true);
				//MiniBot2a.Set(false);
				//MiniBot2b.Set(true);
			}
			if(stick2.GetRawButton(5)) { // top deploy out
				MiniBot2a.Set(true);
				MiniBot2b.Set(false);
			}
			
			
			if(stick2.GetRawButton(6)) { // open
				ClawOpen.Set(true);
				ClawClose.Set(false);
			}
			if(stick2.GetRawButton(8)) { // closed
				ClawOpen.Set(false);
				ClawClose.Set(true);
				ClawLocation += 2;
			}
			
			/*156 straight
			 * 56 90 angle
			 * 10 back
			 */
			
			if(stick2.GetRawButton(1)) { // top peg
				ClawLocation = 156;
				ArmLocation = 105;
			}
			if(stick2.GetRawButton(2)) {
				ClawLocation = 111; // the 90angle / middle peg
				ArmLocation = 50;
			}
			if(stick2.GetRawButton(3)) { // off ground
				ClawLocation = 176;
				ArmLocation = 5;
			}
			if(stick2.GetRawButton(4)) {
				ClawLocation = 0; // back
				ArmLocation = 0;
			}
			
			double tmpClaw = .7*lowClaw(stick2.GetRawAxis(4));
			if(tmpClaw < .2 && tmpClaw > -.2) tmpClaw = 0;
			
			double tmpArm = .4*lowArm(-1*stick2.GetRawAxis(2));
			if(tmpArm < .2 && tmpArm > -.2) tmpArm = 0;
			if(tmpArm > .5) tmpArm = .5;
			if(tmpArm < -.5) tmpArm = -.5;
						
			
			ClawLocation += tmpClaw + tmpArm; // the right joy stick y
			
			if(ClawLocation < 10) ClawLocation = 10;
			if(ClawLocation > 230) ClawLocation = 230;
			
			Claw(ClawLocation);
			
			
			ArmLocation += tmpArm;
			if(ArmLocation > 110) ArmLocation = 110;
			if(ArmLocation < -10)  ArmLocation = -10;
			
			Arm(lowArmLoc(ArmLocation));
			OldArmLocation = ArmLocation;
						
#ifndef NDEBUG
			if(count++%20==0){
			cerr << EncClaw.Get() << '\t' << arm1->GetOutputCurrent() << '\t' << arm1_sec->GetOutputCurrent() << '\t' << ArmLocation << '\t' << EncArm.Get() << endl; 
				//	cerr << arm1->GetOutputCurrent() << '\t' << arm1_sec->GetOutputCurrent() << '\t' << arm2->GetOutputCurrent() << '\t'
			//	 	<< EncArm.Get () << '\t' << LimitArm.Get() << '\t' << EncClaw.Get() << '\t' << LimitClaw.Get() << '\t' << ClawLocation << endl;
				//cerr << '\t' << EncArm.Get() <<'\t' << arm1->Get() << endl;
			//	cerr << Dlf->Get() << '\t' << Dlf->GetSpeed() << '\t' << Dlb->GetSpeed() <<'\t' << Drf->GetSpeed() <<'\t' << Drb->GetSpeed() <<endl;//'\t' << line1.Get() << "\t" << line2.Get() << "\t" << line3.Get() << endl;
			//	cerr << '\t' << Dlb->GetSpeed() << '\t';
			}
#endif			

			if(pneumaticCount++==0) {
				ClawOpen.Set(false);
				ClawClose.Set(false);
				MiniBot1a.Set(false);
				MiniBot1b.Set(false);
				MiniBot2a.Set(false);
				MiniBot2b.Set(false);
			}
			
			Wait(0.01);				// wait for a motor update time
		}
	}
};






////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

class OldRobotDemo : public SimpleRobot {
private:
	CANJaguar *Dlf, *Dlb, *Drb, *Drf;
	Joystick stick;
	lowPass speed, turn, strafe;
public:
	OldRobotDemo (void) :
		stick(1),
		speed(.05),
		turn(.05),
		strafe(.05)
		{
		Wait(3.0);
		Dlf = new CANJaguar(4);
		Dlb = new CANJaguar(5);
		Drf = new CANJaguar(3);
		Drb = new CANJaguar(2);
		}
	void OperatorControl(void) {
		while(!IsDisabled()) {
			GetWatchdog().Feed();
			float speed = stick.GetRawAxis(2);
			float strafe = -1*stick.GetRawAxis(1);
			float turn = -1*stick.GetRawAxis(3);
			Dlf->Set(speed + turn + strafe);
			Dlb->Set(speed + turn - strafe);
			Drf->Set(-speed + turn + strafe);
			Drb->Set(-speed + turn - strafe);
			Wait(.05);	
		}
	}
};

//START_ROBOT_CLASS(OldRobotDemo);

START_ROBOT_CLASS(RobotSystem);
//START_ROBOT_CLASS(PnumaticArmTest);

extern "C" INT32 FRC_ROBOT_START () {
	return FRC_UserProgram_StartupLibraryInit();
}

