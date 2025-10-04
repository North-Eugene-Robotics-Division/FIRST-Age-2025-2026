package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;
import java.util.List;

public class Hardware {

    // Declare OpMode members.
    public LinearOpMode myOpMode = null;
    
    // Declare OpMode members
    public ElapsedTime runtime = new ElapsedTime();

    public boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
  
    //The variable to store our instance of the AprilTag processor.
    public AprilTagProcessor aprilTag;

    //The variable to store our instance of the vision portal.
    public VisionPortal visionPortal;

    // What do we need defined for a Webcam???
    public Webcam webcam;

    //Motors to control all wheels
    public DcMotor LFDrive = null;
    public DcMotor LBDrive = null;
    public DcMotor RFDrive = null;
    public DcMotor RBDrive = null;
    public DcMotor LLinAct = null;
    public DcMotor RLinAct = null;
    public DcMotor LScore  = null;
    public DcMotor RScore  = null;
    
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }
     
    public void init() {
        
        initAprilTag();
        LFDrive = myOpMode.hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        LBDrive = myOpMode.hardwareMap.get(DcMotor.class, "LeftBackDrive");
        RFDrive = myOpMode.hardwareMap.get(DcMotor.class, "RightFrontDrive");
        RBDrive = myOpMode.hardwareMap.get(DcMotor.class, "RightBackDrive");
        LLinAct = myOpMode.hardwareMap.get(DcMotor.class, "LeftLinearActuators");
        RLinAct = myOpMode.hardwareMap.get(DcMotor.class, "RightLinearActuators");
        LScore  = myOpMode.hardwareMap.get(DcMotor.class, "LeftScoringWheel");
        RScore  = myOpMode.hardwareMap.get(DcMotor.class, "RightScoringWheel");
        // Do I need to define the Webcam and Color Sensor here?

        LFDrive.setDirection(DcMotor.Direction.REVERSE);
        LBDrive.setDirection(DcMotor.Direction.REVERSE);
        RFDrive.setDirection(DcMotor.Direction.FORWARD);
        RBDrive.setDirection(DcMotor.Direction.FORWARD);

        // Create initial telemetry to change once the robot starts going
        myOpMode.telemetry.addLine("Driving Information will show here");
        myOpMode.telemetry.addLine("Linear Actuator Information will show here");
        myOpMode.telemetry.addLine("Scoring Motors Data will show here");
        myOpMode.telemetry.addLine("Hinge Door Location will show here");
        myOpMode.telemetry.addLine("Color Sensor Data will show here");
        myOpMode.telemetry.addLine("Webcame Data will show here");
        telemetry.update();
    }
    
    /**
    * Calculates the left/right motor powers required to achieve the requested
    * robot motions: Drive (Axial motion) and Turn (Yaw motion).
    * Then sends these power levels to the motors.
    *
    * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
    * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
    */
     
    double axial;
    double strafe;
    double rotation;
     
    public void driveRobot(double axial, double strafe, double rotation) {
        // Combine drive and turn for blended motion.
        double leftFrontPower  = - axial - strafe - rotation;
        double rightFrontPower = - axial + strafe + rotation;
        double leftBackPower   = - axial + strafe - rotation;
        double rightBackPower  = - axial - strafe + rotation;

        // Scale the values so neither exceed +/- 1.0          
        double max;
        double min;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        min = Math.min(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        min = Math.min(min, Math.abs(leftBackPower));
        min = Math.min(min, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        if (min < -1.0) {
            leftFrontPower    = -1.0;
            rightFrontPower   = -1.0;
            leftBackPower     = -1.0;
            rightBackPower    = -1.0;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        leftFrontWheel.setPower(leftFrontPower);
        rightFrontWheel.setPower(rightFrontPower);
        leftBackWheel.setPower(leftBackPower);
        rightBackWheel.setPower(rightBackPower);
        /*
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
        myOpMode.telemetry.addData("Status", "Run Time: " + runtime.toString());
        myOpMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        myOpMode.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        myOpMode.telemetry.update();
        */
    }
    
    // all parameters should be a string value
    // Start with putting the OpMode NAME, then put in values
    // Put data in this order: driving, linear actuator, scoring, hinge door, color sensor, webcam
    public void telemetryData(String currentOpMode, 
                              String driveData, String linActData, String scoreData, 
                              String hingeDoorData, String colorSensorData, String webcamData {
        // Line order = the order of parameters
        // Insert telemetry data here
    }
}
