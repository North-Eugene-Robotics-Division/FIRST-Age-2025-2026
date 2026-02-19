package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "UniversalDrive", group = "TestCodes")
public class UniversalDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        
        UniversalTestHardware robot = new UniversalTestHardware(this);
        robot.init();
        
        waitForStart();

        while (opModeIsActive()) {
            double Forward  =  gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double Rotation =  gamepad1.right_stick_x;
            double Strafe   =  gamepad1.left_stick_x;
          
            robot.driveRobot(Forward, Rotation, Strafe);
        }
    }
    
}
