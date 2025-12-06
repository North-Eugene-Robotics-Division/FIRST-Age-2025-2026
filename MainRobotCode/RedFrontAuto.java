package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous

public class RedFrontAuto extends LinearOpMode {


    @Override
    public void runOpMode() {
        
        Hardware robot = new Hardware(this);
         
        robot.init();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        robot.driveRobot(1, 0, 0);
        robot.sleep(3000);
        robot.driveRobot(0,-1,0);
        robot.sleep(500);
        robot.chimneyLaunch();
        robot.sleep(200);
        robot.Launcher(1);
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           robot.telemetryData("RedFrontAuto"); 
        }
    }
}
