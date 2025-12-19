package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.M5UltrasonicI2C;

public class Hardware {
	// Declare OpMode members.
	public LinearOpMode myOpMode = null;
	public ElapsedTime runtime = new ElapsedTime();
	public boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
	
	 //Distance sensor
	public M5UltrasonicI2C ultrasonic0;
	public M5UltrasonicI2C ultrasonic1;
	public M5UltrasonicI2C ultrasonic2;
	public M5UltrasonicI2C ultrasonic3;
	
	//The variable to store our instance of the AprilTag processor.
	public AprilTagProcessor aprilTag;

	//The variable to store our instance of the vision portal.
	public VisionPortal visionPortal;

	// What do we need defined for a Webcam???
	public WebcamName webcam;

	public NormalizedColorSensor colorSensor;

	//Motors to control all wheels
	
	//Motors for Driving
	public DcMotor LFDrive = null;
	public DcMotor LBDrive = null;
	public DcMotor RFDrive = null;
	public DcMotor RBDrive = null;
	
	//Motors for Launching
	public DcMotor LLauncher  = null;
	public DcMotor RLauncher  = null;

	//Servos for Chimney
	public Servo intake = null;
	public Servo launchPrimer = null;
	public Servo flipper = null;
	
	//Servos for Intake
	public CRServo LIntake = null;
	public CRServo RIntake = null;

	// All servo positions for the chimney 
	public static final double INTAKE_MIN = .35/5;
	public static final double INTAKE_MIL = .66/5;
	public static final double INTAKE_MID = .55/5;
	public static final double INTAKE_MAX = .75/5;
	public static final double LAUNCH_PRIMER_MIN = .25/5;
	public static final double LAUNCH_PRIMER_MID = .5/5;
	public static final double LAUNCH_PRIMER_MAX = .75/5;
	public static final double FLIPPER_MIN = .5/5;
	public static final double FLIPPER_MAX = 1.0/5;

	public float red, blue, green;
	public float normRed, normBlue, normGreen;

	// Define a constructor that allows the OpMode to pass a reference to itself.
	public Hardware (LinearOpMode opmode) {
		myOpMode = opmode;
	}

	public void init() {

		LFDrive = myOpMode.hardwareMap.get(DcMotor.class, "LeftFrontDrive");
		LBDrive = myOpMode.hardwareMap.get(DcMotor.class, "LeftBackDrive");
		RFDrive = myOpMode.hardwareMap.get(DcMotor.class, "RightFrontDrive");
		RBDrive = myOpMode.hardwareMap.get(DcMotor.class, "RightBackDrive");
		
		LLauncher  = myOpMode.hardwareMap.get(DcMotor.class, "LeftLauncher");
		RLauncher  = myOpMode.hardwareMap.get(DcMotor.class, "RightLauncher");

		intake = myOpMode.hardwareMap.get(Servo.class, "Intake");
		launchPrimer = myOpMode.hardwareMap.get(Servo.class, "LaunchPrimer");
		flipper = myOpMode.hardwareMap.get(Servo.class, "Flipper");
		
		LIntake = myOpMode.hardwareMap.get(CRServo.class, "LeftIntake");
		RIntake = myOpMode.hardwareMap.get(CRServo.class, "RightIntake");

		colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");

		// Motor Directions
		LFDrive.setDirection(DcMotor.Direction.REVERSE);
		LBDrive.setDirection(DcMotor.Direction.REVERSE);
		RFDrive.setDirection(DcMotor.Direction.FORWARD);
		RBDrive.setDirection(DcMotor.Direction.FORWARD);

		LLauncher.setDirection(DcMotor.Direction.REVERSE);
		RLauncher.setDirection(DcMotor.Direction.FORWARD);

		//Servos are between 0 and 1, and has 5 total rotations. to go between one rotation, divide by 5 (never do 0)

		//Default positions for the chimney servos is straight downwards, excluding the flipper, which has less of a needed starting position
		intake.setPosition(INTAKE_MID);
		launchPrimer.setPosition(LAUNCH_PRIMER_MID);
		flipper.setPosition(FLIPPER_MIN);

		// Create initial telemetry
		myOpMode.telemetry.addLine("Driving Information will show here");
		myOpMode.telemetry.addLine("Scoring Motor Data will show here");
		myOpMode.telemetry.addLine("Chimney Servo positions will show here");
		myOpMode.telemetry.addLine("Intake CRServo positions will show here");
		myOpMode.telemetry.addLine("Color Sensor Data will show here");
		myOpMode.telemetry.addLine("Webcam Data will show here");
		myOpMode.telemetry.update();

		initAprilTag();
		ultrasonic0 = new M5UltrasonicI2C(myOpMode, 0);
		ultrasonic0.init();
		ultrasonic1 = new M5UltrasonicI2C(myOpMode, 1);
		ultrasonic1.init();
		ultrasonic2 = new M5UltrasonicI2C(myOpMode, 2);
		ultrasonic2.init();
		ultrasonic3 = new M5UltrasonicI2C(myOpMode, 3);
		ultrasonic3.init();
	}


	// parameters are all doubles
	// data: from auto/gamepad, to drive motors
	public void driveRobot(double forward, double rotation, double strafe) {
		// Combine drive and turn for blended motion.
				double leftFrontPower  =   forward - strafe - rotation;
				double rightFrontPower =   forward + strafe + rotation;
				double leftBackPower   =  - forward - strafe + rotation;
				double rightBackPower  =  - forward + strafe - rotation;
				
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
					leftFrontPower	= -1.0;
					rightFrontPower   = -1.0;
					leftBackPower	 = -1.0;
					rightBackPower	= -1.0;
				}
			
				// Send calculated power to wheels
				LFDrive.setPower(leftFrontPower);
				RFDrive.setPower(rightFrontPower);
				LBDrive.setPower(leftBackPower);
				RBDrive.setPower(rightBackPower);
	}


	public void initAprilTag() {
	// Create the AprilTag processor.
		aprilTag = new AprilTagProcessor.Builder()
				.setDrawTagOutline(true)
				.setDrawAxes(true)
				.setDrawCubeProjection(true)
				.setDrawTagID(true)
//			  .setOutputUnits(AprilTagProcessor.OutputUnits.INCHES)  // or METERS if preferred
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

	public void getDetectedColor() {
		
		normRed = colorSensor.getNormalizedColors().red	 / colorSensor.getNormalizedColors().alpha;
		normBlue = colorSensor.getNormalizedColors().blue   / colorSensor.getNormalizedColors().alpha;
		normGreen = colorSensor.getNormalizedColors().green / colorSensor.getNormalizedColors().alpha;
	}
	
	public String readAprilTag (String Task) {

		List<AprilTagDetection> currentDetections = aprilTag.getDetections();
		//telemetry.addData("# AprilTags Detected", currentDetections.size());

		// Step through the list of detections and display info for each one.
		for (AprilTagDetection detection : currentDetections) {
			if (detection.metadata != null) {
				if (Task == "ID") {
					return String.format(""+detection.id );
				} else if  (Task == "Name") {
					return String.format(detection.metadata.name);
				} else {
					return "Unknown Task";
				}
			} else {
				return "Unknown";
			}
		}   // end for() loop

	return "None";
	}
	
	public void sleep(int millis) {
		try {
			Thread.sleep(millis);
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
		}
	}

	public void chimneyLaunch() {
		launchPrimer.setPosition(LAUNCH_PRIMER_MAX);
		sleep(700);
		intake.setPosition(INTAKE_MIN);
		sleep(700);
		LLauncher.setPower(1);
		launchPrimer.setPosition(LAUNCH_PRIMER_MIN);
		sleep(1200);
		intake.setPosition(INTAKE_MID);
		launchPrimer.setPosition(LAUNCH_PRIMER_MID);
		LLauncher.setPower(0);
	}

	// public void chimneyRecycleToggle() {
	//	 if (recycling == false) { 
	//		 recycling = true;
	//		 flipper.setPosition(FLIPPER_MIN);
	//	 } else if (recycling == true) {
	//		 recycling = false;
	//		 flipper.setPosition(FLIPPER_MAX);
	//	 }
	// }

	public void Launcher(double Power) {
		LLauncher.setPower(Power);
		RLauncher.setPower(Power);
	}
	
	public void Intake() {
		intake.setPosition(INTAKE_MAX);
		sleep(200);
		LIntake.setPower(-1);
		RIntake.setPower(1);
	}
	
	public void Normalize() {
		LIntake.setPower(0);
		RIntake.setPower(0);
		sleep(50);
		intake.setPosition(INTAKE_MID);
		launchPrimer.setPosition(LAUNCH_PRIMER_MID);
	}
	
	public void Eject() {
		LIntake.setPower(1);
		RIntake.setPower(-1);
		intake.setPosition(INTAKE_MAX);
		sleep(100);
		launchPrimer.setPosition(LAUNCH_PRIMER_MAX);
	}

	
	
	// Only parameter is the op mode
	// All other data is inside, or given to, this file
	// Run this whenever an update is wanted INSIDE of the op mode
	public void telemetryData(String currentOpMode) {
		//Says Op Mode name
		myOpMode.telemetry.addLine("OpMode Being Ran: " + currentOpMode);
		//Says value between -1.0 and 1.0 for each motor
		myOpMode.telemetry.addData("Drive Powers: ", String.format("LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f", LFDrive.getPower(), RFDrive.getPower(), LBDrive.getPower(), RBDrive.getPower()));
		myOpMode.telemetry.addData("Left Launcher: " + LLauncher.getPower(), "Right Launcher: " + RLauncher.getPower());
		myOpMode.telemetry.addData("Chimney Servo Positions: ", String.format("Intake: %.2f, Launch Primer: %.2f, Flipper: %.2f", intake.getPosition(), launchPrimer.getPosition(), flipper.getPosition()));
		myOpMode.telemetry.addData("Intake CRServo Powers: ", String.format("Left Intake: %.2f, Right Intake: %.2f, ", intake.getPosition(), launchPrimer.getPosition(), flipper.getPosition()));
		myOpMode.telemetry.addData("Colors: ", "Red: %.2f, Blue: %.2f, Green: %.2f", normRed, normBlue, normGreen);
		myOpMode.telemetry.addLine("Webcam: " + readAprilTag("Name"));
		myOpMode.telemetry.addData("Ultrasonic (cm)", "%.1f", ultrasonic0.getDistanceCm());
		myOpMode.telemetry.addData("Ultrasonic2 (cm)", "%.1f", ultrasonic1.getDistanceCm());
		myOpMode.telemetry.addData("Ultrasonic3 (cm)", "%.1f", ultrasonic2.getDistanceCm());
		myOpMode.telemetry.addData("Ultrasonic4 (cm)", "%.1f", ultrasonic3.getDistanceCm());

		myOpMode.telemetry.update();
	}
	
	
}
