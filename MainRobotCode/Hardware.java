package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

    public ElapsedTime runtime = new ElapsedTime();

    public boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    
    //The variable to store our instance of the AprilTag processor.
    public AprilTagProcessor aprilTag;

    //The variable to store our instance of the vision portal.
    public VisionPortal visionPortal;

    // What do we need defined for a Webcam???
    public WebcamName webcam;

    public ColorSensor colorSensor;

    //Motors to control all wheels
    public DcMotor LFDrive = null;
    public DcMotor LBDrive = null;
    public DcMotor RFDrive = null;
    public DcMotor RBDrive = null;
    public DcMotor LLinAct = null;
    public DcMotor RLinAct = null;
    public DcMotor LScore  = null;
    public DcMotor RScore  = null;

    public Servo intake = null;
    public Servo launchPrimer = null;
    public Servo flipper = null;

    // All servo positions for the chimney 
    public static final double INTAKE_MIN = .755/5;
    public static final double INTAKE_MIL = .66/5;
    public static final double INTAKE_MID = .55/5;
    public static final double INTAKE_MAX = .33/5;
    public static final double LAUNCH_PRIMER_MIN = .25/5;
    public static final double LAUNCH_PRIMER_MID = .5/5;
    public static final double LAUNCH_PRIMER_MAX = 1.0/5;
    public static final double FLIPPER_MIN = .5/5;
    public static final double FLIPPER_MAX = 1.0/5;

    public boolean recycling = false;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

        LFDrive = myOpMode.hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        LBDrive = myOpMode.hardwareMap.get(DcMotor.class, "LeftBackDrive");
        RFDrive = myOpMode.hardwareMap.get(DcMotor.class, "RightFrontDrive");
        RBDrive = myOpMode.hardwareMap.get(DcMotor.class, "RightBackDrive");
        LLinAct = myOpMode.hardwareMap.get(DcMotor.class, "LeftLinearActuators");
        RLinAct = myOpMode.hardwareMap.get(DcMotor.class, "RightLinearActuators");
        LScore  = myOpMode.hardwareMap.get(DcMotor.class, "LeftScoringWheel");
        RScore  = myOpMode.hardwareMap.get(DcMotor.class, "RightScoringWheel");

        intake = myOpMode.hardwareMap.get(Servo.class, "intake");
        launchPrimer = myOpMode.hardwareMap.get(Servo.class, "launch_primer");
        flipper = myOpMode.hardwareMap.get(Servo.class, "flipper");

        colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "ColorSensor");

        // Motor Directions
        LFDrive.setDirection(DcMotor.Direction.REVERSE);
        LBDrive.setDirection(DcMotor.Direction.REVERSE);
        RFDrive.setDirection(DcMotor.Direction.FORWARD);
        RBDrive.setDirection(DcMotor.Direction.FORWARD);

        //Servos are between 0 and 1, and has 5 total rotations. to go between one rotation, divide by 5 (never do 0)

        //Default positions for the chimney servos is straight downwards, excluding the flipper, which has less of a needed starting position
        intake.setPosition(INTAKE_MID);
        launchPrimer.setPosition(LAUNCH_PRIMER_MID);
        flipper.setPosition(FLIPPER_MIN);

        // Create initial telemetry
        myOpMode.telemetry.addLine("Driving Information will show here");
        myOpMode.telemetry.addLine("Linear Actuator Information will show here");
        myOpMode.telemetry.addLine("Scoring Motors Data will show here");
        myOpMode.telemetry.addLine("Intake position will show here");
        myOpMode.telemetry.addLine("Launch primer position will show here");
        myOpMode.telemetry.addLine("Flipper position will show here");
        myOpMode.telemetry.addLine("Color Sensor Data will show here");
        myOpMode.telemetry.addLine("Webcam Data will show here");
        myOpMode.telemetry.update();

        initAprilTag();
    }


    // parameters are all doubles
    // data: from auto/gamepad, to drive motors
    public void driveRobot(double forward, double rotation, double strafe) {
        // Combine drive and turn for blended motion.
        double leftFrontPower  =   forward + strafe - rotation;
        double rightFrontPower = - forward + strafe + rotation;
        double leftBackPower   = - forward + strafe - rotation;
        double rightBackPower  =   forward + strafe + rotation;
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
        LFDrive.setPower(leftFrontPower);
        RFDrive.setPower(rightFrontPower);
        LBDrive.setPower(leftBackPower);
        RBDrive.setPower(rightBackPower);
    }

    // public void liftRobot(String Direction) {
    //     switch (Direction){
    //         case "Up": 
    //             LLinAct.setPower(1);
    //             RLinAct.setPower(1);
    //             break;
    //         case "Down": 
    //             LLinAct.setPower(-1);
    //             RLinAct.setPower(-1);
    //             break;
    //         }
    // }
    
    // public void stopRobotLift(){
    //     LLinAct.setPower(0);
    //     RLinAct.setPower(0);
    // }

    public void initAprilTag() {
    // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
//              .setOutputUnits(AprilTagProcessor.OutputUnits.INCHES)  // or METERS if preferred
                .build();

    // Select camera: webcam or phone
    if (USE_WEBCAM) {
         webcam = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    // Build the Vision Portal using the camera and AprilTag processor
    VisionPortal.Builder builder = new VisionPortal.Builder();

    if (USE_WEBCAM) {
        builder.setCamera(webcam);
    } else {
        builder.setCamera(BuiltinCameraDirection.BACK);
    }

    builder.addProcessor(aprilTag);

    // Create the portal (this actually starts the camera stream)
    visionPortal = builder.build();

    // Optional: add initial telemetry to confirm initialization
    myOpMode.telemetry.addLine("AprilTag vision initialized.");
    myOpMode.telemetry.update();
    }
    
    public void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void chimneyLaunch() {
        intake.setPosition(INTAKE_MIL);
        sleep(1000);
        launchPrimer.setPosition(LAUNCH_PRIMER_MIN);
        sleep(1000);
        intake.setPosition(INTAKE_MIN);
        sleep(1000);
        intake.setPosition(INTAKE_MID);
        sleep(500);
        launchPrimer.setPosition(LAUNCH_PRIMER_MID);
    }

    public void chimneyRecycle() {
        if (recycling == false) { 
            recycling = true;
            flipper.setPosition(FLIPPER_MIN);
        } else if (recycling == true) {
            recycling = false;
            flipper.setPosition(FLIPPER_MAX);
        }
    }
    
    
    // Only parameter is the op mode
    // All other data is inside, or given to, this file
    // Run this whenever an update is wanted INSIDE of the op mode
    public void telemetryData(String currentOpMode) {
        //Says Op Mode name
        myOpMode.telemetry.addLine("OpMode Being Ran: " + currentOpMode);
        //Says value between -1.0 and 1.0 for each motor
        myOpMode.telemetry.addData("Drive Power", 
                                   String.format("LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f",
                                   LFDrive.getPower(), RFDrive.getPower(), LBDrive.getPower(), RBDrive.getPower()));
        myOpMode.telemetry.addLine("LeftLinAct: " + LLinAct.getPower() + 
                                   " RightLinAct: " + RLinAct.getPower());
        myOpMode.telemetry.addLine("LeftScoring: " + LScore.getPower() + 
                                   " RightScoring: " + RScore.getPower());
        myOpMode.telemetry.addLine("Intake: " + intake.getPosition());
        myOpMode.telemetry.addLine("Launch Primer: " + launchPrimer.getPosition());
        myOpMode.telemetry.addLine("Flipper: " + flipper.getPosition());
        myOpMode.telemetry.addLine("Color Sensor: " + colorSensor);
        myOpMode.telemetry.addLine("Webcam: " + webcam);
        myOpMode.telemetry.update();
    }
    
    
}
