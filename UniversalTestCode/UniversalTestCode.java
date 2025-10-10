package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.UniversalTestHardware;

@TeleOp(name = "TestMotors+Servos", group = "TestCodes")
public class UniversalTestCode extends LinearOpMode {
    
    @Override
    public void runOpMode() {
       
        UniversalTestHardware robot = new UniversalTestHardware(this);
        
        robot.init();
        
        telemetry.addData("MotorsList", robot.motors.length);
        telemetry.addData("Active motor", robot.motorsNamesList[robot.motorNumber]);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            if (gamepad1.right_bumper){
                robot.cycleMotorPos();
                robot.sleep(250);
            }
            if (gamepad1.left_bumper){
                robot.cycleMotorNeg();
                robot.sleep(250);
            }
            
            if (gamepad1.a){
                robot.motorPowerPos();
            }
            if (gamepad1.b){
                robot.motorPowerNeg();
            }
            
            // if (gamepad1.x){
            //     robot.powerMotor();
            // }
            // if (gamepad1.x){
            //     robot.turnServoPos();
            // }
            // if (gamepad1.y){
            //     robot.turnServoNeg();
            // }

        }
    }
}
