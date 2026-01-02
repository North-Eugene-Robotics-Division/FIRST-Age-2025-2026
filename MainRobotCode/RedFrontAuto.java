package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Collections;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous
public class RedFrontAuto extends LinearOpMode {
	ArrayList<String> Order = new ArrayList<>();
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
			robot.telemetryData("RedFrontAuto");
			if (ran == false) {
				robot.driveRobot(1, 0, 0, "RedAuto");
				robot.sleep(3000);
				robot.driveRobot(0,0,0, "RedAuto");
				int tries = 0;
				
				//First Try
				switch (robot.readAprilTag("ID")) {
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
						robot.driveRobot(0,5,0, "RedAuto");
						robot.sleep(500);
						
						//Second Try
						switch (robot.readAprilTag("ID")) {
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
								robot.driveRobot(0,5,0, "RedAuto");
								robot.sleep(100);
								
								//Third Try
								switch (robot.readAprilTag("ID")) {
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
										robot.driveRobot(0,-5,0, "RedAuto");
										robot.sleep(800);
										Collections.addAll(Order, "F", "A", "I", "L");
										break;
								}
								break;
						}
						break;
				}
				robot.driveRobot(0,1,0, "RedAuto");
				robot.sleep(250);
				robot.driveRobot(1,0,0, "RedAuto");
				robot.sleep(800);
				robot.driveRobot(0,-1,0, "RedAuto");
				robot.sleep(1000);
				robot.driveRobot(0,0,0, "RedAuto");
				
				if (Order.get(0) == "F")	{
					break;
				} else {
					for (int step = 0; step > 4;){
						if (robot.readArtifactColor() != Order.get(step)) {
							robot.chimneyRecycleToggle();
							robot.Launcher(1);
							robot.sleep(500);
							robot.Launcher(0);
						} else {
							//launch the correct ball
							robot.Launcher(1);
							robot.sleep(500);
							robot.Launcher(0);
							step++;
						}
					}
				}
				ran = true;
			}
			robot.telemetryData("RedFrontAuto"); 
			telemetry.addLine(Order.toString());
			telemetry.update();
		}
	}
}
