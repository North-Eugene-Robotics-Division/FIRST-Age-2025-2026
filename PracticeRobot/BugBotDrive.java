package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="BugBot", group="Bug-Bot")
public class BugBotDrive extends LinearOpMode {
    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    private TouchSensor limitButton = null;
    private boolean isArmDown = false;
    
    private void resetArm() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
    }
        
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        leftDrive  = hardwareMap.get(DcMotor.class, "Left_motor");
        rightDrive = hardwareMap.get(DcMotor.class, "Right_motor");
        arm = hardwareMap.get(DcMotor.class, "arm");
        limitButton = hardwareMap.get(TouchSensor.class, "limit_Button"); 
        
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        resetArm();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
           
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double dropMultiplier = 1.0;

            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0);
            rightPower   = Range.clip(drive - turn, -1.0, 1.0);
            
            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            
            if (arm.getCurrentPosition() >= -150) {
                dropMultiplier = 0.25;
            } else {
                dropMultiplier = 1.0;
            }
            
            double TRIGGER_TOLERANCE = 0.2d;
            if (gamepad1.right_trigger > TRIGGER_TOLERANCE && (arm.getCurrentPosition() > -470)) {
                arm.setPower(-gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > TRIGGER_TOLERANCE) {
                arm.setPower(gamepad1.left_trigger * dropMultiplier);
            } else if (arm.getCurrentPosition() < -500) {
                arm.setPower(0.001);
            } else {
                arm.setPower(0.0f);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Arm", "position: " + arm.getCurrentPosition());
            
            if (limitButton.isPressed() && !isArmDown) {
                telemetry.addData("Button Status", "Pressed");
                resetArm();
                isArmDown = true;
            } else if (!limitButton.isPressed()) {
                telemetry.addData("Button Status", "Not Pressed");
                isArmDown = false;
            } else {
                telemetry.addData("Button Status", "Pressed");
                resetArm();
            }
            telemetry.update();
        }
    }
}
