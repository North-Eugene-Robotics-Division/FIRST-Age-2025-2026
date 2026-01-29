package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name="Drive_Code", group="Linear OpMode")
public class DriveCode extends LinearOpMode {
	private ElapsedTime runtime = new ElapsedTime();
	
	private double SpeedModifier = 1;

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
			
			// POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
			double Forward  =  -gamepad1.left_stick_y * SpeedModifier;  
			double Rotation =  gamepad1.right_stick_x * SpeedModifier;
			double Strafe   =  gamepad1.left_stick_x * SpeedModifier;
			robot.driveRobot(Forward, Rotation, Strafe, "DriveCode");
			
			// Triggers used to launch the artifact
			double LaunchPowerPos = gamepad2.right_trigger;
			double LaunchPowerNeg = -gamepad2.left_trigger;
			
			// Buttons which change the robot speed
			
			// X on Driving Gamepad inverts the controls
			if (gamepad1.xWasReleased()) {
				SpeedModifier *= -1;
			}
			
			//dpad up and down increase or decrease the robot speed
			if (gamepad1.dpadUpWasReleased()) {
				if (SpeedModifier > 0) {
					SpeedModifier += .1;
				} else {
					SpeedModifier += -.1;
				}
			}
			
			if (gamepad1.dpadDownWasReleased()) {
				if (SpeedModifier > 0) {
					SpeedModifier += -.1;
				} else {
					SpeedModifier += .1;
				}
			}
			
			//Speed Modifier Regulation
			if (SpeedModifier < -1) {
				SpeedModifier = -1;
			}
			
			if (SpeedModifier > 1) {
				SpeedModifier = 1;
			}
			
		
			// Button Functions
			if (gamepad2.yWasReleased()) {
				robot.chimneyRecycleToggle();
			}
			
			if (gamepad2.aWasReleased()) {
				robot.chimneyLaunch();
			}
			
			if (gamepad2.dpadLeftWasPressed()) {
				robot.Normalize();
			}
			
			if (gamepad2.xWasPressed()) {
				robot.Intake();
			}
			
			if (gamepad2.xWasReleased()) {
				robot.Normalize();
			}
			
			if (gamepad2.bWasPressed()) {
				robot.Eject();
			}
			 
			if (gamepad2.bWasReleased()) {
				robot.Normalize();
			}
			
			if (gamepad2.right_trigger > 0) {
				robot.Launcher(LaunchPowerPos);
			}
			
			
			if (gamepad2.left_trigger > 0) {
				robot.Launcher(LaunchPowerNeg);
			}
			
			robot.getDetectedColor();
			// robot.ultrasonic0.update();
			// robot.ultrasonic1.update();
			// robot.ultrasonic2.update();
			// robot.ultrasonic3.update();

			robot.telemetryData("DriveCode");
		}
	} 
}
