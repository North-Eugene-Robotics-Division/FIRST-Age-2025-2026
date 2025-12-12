package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name="Drive_Code", group="Linear OpMode")

public class DriveCode extends LinearOpMode /*implements Runnable*/ {
 
 private ElapsedTime runtime = new ElapsedTime();
 // public void run(){
 //   double Forward  =  gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
 //   double Rotation =  gamepad1.right_stick_x;
 //   double Strafe   =  gamepad1.left_stick_x;
 //   robot.driveRobot(Forward, Rotation, Strafe);
 // }
 
 // DriveCode runnable = new Drivecode();
 // Thread thread = new Thread(runnable);
 // boolean threadMade = false;
 // public void test(){
 //  if (threadMade) return;
 //  thread.start();
 // }
 
 @Override
 public void runOpMode() {
	 
	 Hardware robot = new Hardware(this);
 
	 robot.init();

	 
	 telemetry.addData("Status", "Initialized");
	 telemetry.update();

	 waitForStart();
	 runtime.reset();

	 // run until the end of the match (driver presses STOP)
	 while (opModeIsActive()) {
		 // test();
		 // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
		 
		 double Forward  =  gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
		 double Rotation =  gamepad1.right_stick_x;
		 double Strafe   =  gamepad1.left_stick_x;
		 robot.driveRobot(Forward, Rotation, Strafe);
		 
		 double LaunchPowerPos = gamepad2.right_trigger;
		 double LaunchPowerNeg = -gamepad2.left_trigger;
		 
		
		 
		 if (gamepad2.aWasReleased()) {
			 robot.chimneyLaunch();
		 }
		 
		 if (gamepad2.xWasPressed()) {
			 robot.Intake();
		 }
		 
		 if (gamepad1.xWasReleased()) {
			 robot.Normalize();
		 }
		 
		 if (gamepad2.bWasPressed()) {
			 robot.Eject();
		 }
		 
		 if (gamepad1.bWasReleased()) {
			 robot.Normalize();
		 }
		 
		 if (gamepad2.right_trigger > 0) {
			 robot.Launcher(LaunchPowerPos);
		 }
		 
		 if (gamepad2.left_trigger > 0) {
			 robot.Launcher(LaunchPowerNeg);
		 }
		
		//  if (gamepad1.xWasPressed()) {
		//	  robot.chimneyRecycleToggle();
		//  }
		 
		 robot.getDetectedColor();
		 robot.telemetryData("DriveCode");
		 
	 }
 } 
}
