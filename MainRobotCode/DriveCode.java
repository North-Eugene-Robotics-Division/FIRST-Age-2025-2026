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
         double Forward  =  gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
         double Rotation =  gamepad1.right_stick_x;
         double Strafe   =  gamepad1.left_stick_x;
         
         robot.driveRobot(Forward, Rotation, Strafe);

         if gamepad1.bWasPressed()) {
            robot.Launcher("ON");
         }
         
         if gamepad1.bWasReleased()) {
            robot.Launcher("OFF");
         }
         
         if (gamepad1.aWasPressed()) {
             robot.chimneyLaunch();
         }
         
         if (gamepad1.xWasPressed()) {
             robot.chimneyRecycle();
         }
         
         robot.telemetryData("DriveCode");
     }
 } 
}
