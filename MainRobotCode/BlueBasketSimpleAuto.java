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

public class BlueBasketSimpleAuto extends LinearOpMode {


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
			robot.driveRobot(-1,0,0,"BlueBasketSimpleAuto"); 
			robot.sleep(200);
			
			
			robot.telemetryData("BlueBasketSimpleAuto"); 
			telemetry.update();

		}
	}
}
