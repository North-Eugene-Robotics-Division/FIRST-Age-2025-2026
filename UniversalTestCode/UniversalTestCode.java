package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "UniversalTestCode", group = "TestCodes")
public class UniversalTestCode extends LinearOpMode {

    @Override
    public void runOpMode() {

        UniversalTestHardware robot = new UniversalTestHardware(this);
        robot.init();
        boolean servoMode = false;
        telemetry.addData("Motors", robot.motors.length);
        telemetry.addData("Servos", robot.servos.length);
        telemetry.addData("Active motor", robot.motorsNamesList[robot.motorNumber]);
        telemetry.addData("Active servo", robot.servosNamesList[robot.servoNumber]);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (servoMode) {
                telemetry.addLine("Servo Mode");
            } else {
                telemetry.addLine("Motor Mode");
            }

            // Cycle selected motor
            if (gamepad1.right_bumper) {
                if (servoMode) {
                    robot.cycleServoPos();
                } else {
                    robot.cycleMotorPos();
                }
                robot.sleep(250);
            }
            if (gamepad1.left_bumper) {
                if (servoMode) {
                    robot.cycleServoNeg();
                } else {
                    robot.cycleMotorNeg();
                }
                robot.sleep(250);
            }
            telemetry.addLine("Active motor" + robot.motorsNamesList[robot.motorNumber]);
            telemetry.addLine("Active servo" + robot.servosNamesList[robot.servoNumber]);

            // read buttons
            if (gamepad1.a) {
                if (servoMode) {
                    //servo code here
                    robot.setServoPos();
                    telemetry.addLine("Set" + robot.servosNamesList[robot.servoNumber] + " to maximum position");
                } else {
                    //motor code here
                    robot.motorPowerPos();
                    telemetry.addLine("Set" + robot.motorsNamesList[robot.motorNumber] + " to 1");
                }
            }
            if (gamepad1.b) {
                if (servoMode) {
                    //servo code here
                    robot.setServoNeg();
                    telemetry.addLine("Set" + robot.servosNamesList[robot.servoNumber] + " to minimum position");
                } else {
                    //motor code here
                    robot.motorPowerNeg();
                    telemetry.addLine("Set" + robot.motorsNamesList[robot.motorNumber] + " to -1");
                }
            }

            if (gamepad1.y) {
                //should toggle, but maybe not.
                servoMode = !servoMode;
                robot.sleep(250);
            }
            //telemetry.addLine(String.valueOf(servoMode));

            //stop
            if (gamepad1.x) {
                if (servoMode) {
                    //servo code here
                    robot.servoStop();
                    telemetry.addLine("Set" + robot.servosNamesList[robot.servoNumber] + " to current position");
                } else {
                    //motor code here
                    robot.motorPowerStop();
                    telemetry.addLine("Set" + robot.motorsNamesList[robot.motorNumber] + " to 0");
                }
            }
            telemetry.update();
        }
    }
}
