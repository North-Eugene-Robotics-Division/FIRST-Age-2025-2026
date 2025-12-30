package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous

public class BlueFrontAuto extends LinearOpMode {
    List<String> Order = new ArrayList<>();
    int Step = 0;
	boolean ran = false;

	@Override
	public void runOpMode() {
		
		Hardware robot = new Hardware(this);
		 
		robot.init();
		
		telemetry.addData("Status", "Initialized");
		telemetry.update();
		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {
			if (ran == false) {
								
				robot.driveRobot(1, 0, 0);
				robot.sleep(3000);
				robot.driveRobot(0,0,0);
				switch (readAprilTag("ID")) {
					case "21": 
						//21 is GPP
						Collections.addAll(Order, "G", "P", "P");
						break;
					case "22": 
						//22 is PGP
						Collections.addAll(Order, "P", "G", "P");
						break;
					case "23":
						//23 is PPG
						 Collections.addAll(Order, "P", "P", "G");
						break;
					default:
						//Just guess
						getOrder();
						break;
				}
		        robot.driveRobot(0,-1,0);
		        robot.sleep(250);
		        robot.driveRobot(1,0,0);
		        robot.sleep(800);
		        robot.driveRobot(0,1,0);
		        robot.sleep(1000);
						
				ran = true;
			}
		   robot.telemetryData("BlueFrontAuto"); 
		}
	}
}
