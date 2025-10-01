package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PracticeRobotHardware;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")

public class RobotDrive extends LinearOpMode {
    
    PracticeRobotHardware robot = new PracticeRobotHardware(this);
    
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        
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
            
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            // telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            // telemetry.update();
        }
    } 
}
