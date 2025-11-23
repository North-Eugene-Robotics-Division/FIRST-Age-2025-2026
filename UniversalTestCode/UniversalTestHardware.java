package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class UniversalTestHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor motor0   = null;
    private DcMotor motor1   = null;
    private DcMotor motor2   = null;
    private DcMotor motor3   = null;
    private DcMotor motor4   = null;
    private DcMotor motor5   = null;
    private DcMotor motor6   = null;
    private DcMotor motor7   = null;

    private Servo servo00  = null;
    private Servo servo01  = null;
    private Servo servo02  = null;
    private Servo servo03  = null;
    private Servo servo04  = null;
    private Servo servo05  = null;
    private Servo servo06  = null;
    private Servo servo07  = null;
    private Servo servo08  = null;
    private Servo servo09  = null; //limited to 10 servos for decode season
    private Servo servo10  = null;
    private Servo servo11  = null;

    public String[] telemetryMsg = {};
    public DcMotor[] motors;
    public String[] motorsNamesList = {"Motor Port 0", "Motor Port 1", "Motor Port 2", "Motor Port 3", "Motor Port 4", "Motor Port 5", "Motor Port 6", "Motor Port 7"};

    public Servo[] servos;
    //changed this, was calld stringNamesList and was just the motor list with the first one changed to servo.
    public String[] servosNamesList = {"Servo Port 0", "Servo Port 1", "Servo Port 2", "Servo Port 3", "Servo Port 4", "Servo Port 5", "Servo Port 6", "Servo Port 7", "Servo Port 8", "Servo Port 9", "Servo Port 10", "Servo Port 11"};

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public UniversalTestHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }


    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        motor0  = myOpMode.hardwareMap.get(DcMotor.class, "motor0");
        motor1  = myOpMode.hardwareMap.get(DcMotor.class, "motor1");
        motor2  = myOpMode.hardwareMap.get(DcMotor.class, "motor2");
        motor3  = myOpMode.hardwareMap.get(DcMotor.class, "motor3");
        motor4  = myOpMode.hardwareMap.get(DcMotor.class, "motor4");
        motor5  = myOpMode.hardwareMap.get(DcMotor.class, "motor5");
        motor6  = myOpMode.hardwareMap.get(DcMotor.class, "motor6");
        motor7  = myOpMode.hardwareMap.get(DcMotor.class, "motor7");

        // Define and initialize ALL installed servos.
        servo00 = myOpMode.hardwareMap.get(Servo.class, "servo00");
        servo01 = myOpMode.hardwareMap.get(Servo.class, "servo01");
        servo02 = myOpMode.hardwareMap.get(Servo.class, "servo02");
        servo03 = myOpMode.hardwareMap.get(Servo.class, "servo03");
        servo04 = myOpMode.hardwareMap.get(Servo.class, "servo04");
        servo05 = myOpMode.hardwareMap.get(Servo.class, "servo05");
        servo06 = myOpMode.hardwareMap.get(Servo.class, "servo06");
        servo07 = myOpMode.hardwareMap.get(Servo.class, "servo07");
        servo08 = myOpMode.hardwareMap.get(Servo.class, "servo08");
        servo09 = myOpMode.hardwareMap.get(Servo.class, "servo09");
        servo10 = myOpMode.hardwareMap.get(Servo.class, "servo10");
        servo11 = myOpMode.hardwareMap.get(Servo.class, "servo11");

        //filling the arrays made at the top of class with the motor references.
        motors = new DcMotor[]{motor0, motor1, motor2, motor3, motor4, motor5, motor6, motor7};
        servos = new Servo[]{servo00, servo01, servo02, servo03, servo04, servo05, servo06, servo07, servo08, servo09, servo10, servo11};


        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    // ------ Motors Start ----------- \\
    public int motorNumber = 0;
    public String currentMotor = "";
    public double motorSpeed = 1.0;
    
    //changes the motor in use to the next in line.
    public void cycleMotorPos() {
        motorNumber = (motorNumber + 1)%8;
    }

    //changes the motor in use to the previous one.
    public void cycleMotorNeg() {
        motorNumber = (motorNumber - 1);
        if (motorNumber == -1) {
            motorNumber = 7;
        }
    }
    public void motorPowerStop() {
        motors[motorNumber].setPower(0);
    }

    public void motorPowerPos() {
        motors[motorNumber].setPower(motorSpeed);
    }

    public void motorPowerNeg() {
        motors[motorNumber].setPower(-motorSpeed);
    }
    
    public void motorPowerUp() {
        motorSpeed += 0.05;
        if (motorSpeed > 1.0) {
            motorSpeed = 1.0;
        }
    }
    
    public void motorPowerDown() {
        motorSpeed -= 0.05;
        if (motorSpeed < 0.0) {
            motorSpeed = 0.0;
        }
    }
    
    private double roundedMotorSpeed() {
        double roundedMotorSpeed = Math.round(motorSpeed * 100.0) / 100.0;
        return roundedMotorSpeed;
    }
//-------- Motors End --------\\


// ----- Servos Start ------- \\
    //not sure how to do the powering and stuff, so I'm just doing the cycling.
    public int servoNumber = 0;
    public String currentServo = "";

    public void cycleServoPos() {
        servoNumber = (servoNumber + 1)%12;
    }

    public void cycleServoNeg() {
        servoNumber = (servoNumber - 1);
        if (servoNumber == - 1) {
            servoNumber = 11;
        }
    }

    public void setServoMax() {
        servos[servoNumber].setPosition(Servo.MAX_POSITION);
    }
    
    public void setServoMin() {
        servos[servoNumber].setPosition(Servo.MIN_POSITION);
    }
    public void servoStop() {
        //Use 0.5 to either set a servo 
        servos[servoNumber].setPosition(0.5);
    }

// ------ Servos End ------- \\

// ------ tools ------- \\
    public void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    
     public void telemetryData(String currentOpMode) {
        //Says Op Mode name
        myOpMode.telemetry.addLine("OpMode Being Ran: " + currentOpMode);
        //Says value between -1.0 and 1.0 for each motor
        myOpMode.telemetry.addLine("Active motor" + motorsNamesList[motorNumber]);
        myOpMode.telemetry.addLine("Active servo" + servosNamesList[servoNumber]);
        myOpMode.telemetry.addData("Drive Power", 
                                    String.format("/n M0: %.2f \n M1: %.2f \n M2: %.2f \n M3: %.2f \n M4: %.2f \n M5: %.2f \n M6: %.2f \n M7: %.2f",
                                    motor0.getPower(), motor1.getPower(), motor2.getPower(), motor3.getPower(), motor4.getPower(), motor5.getPower(), motor6.getPower(), motor7.getPower()));
        myOpMode.telemetry.addLine("Motor Speed" + roundedMotorSpeed());
        
    //     myOpMode.telemetry.addLine("LeftLinAct: " + LLinAct.getPower() + 
    //                                 " RightLinAct: " + RLinAct.getPower());
    //     myOpMode.telemetry.addLine("LeftScoring: " + LScore.getPower() + 
    //                                 " RightScoring: " + RScore.getPower());
    //     myOpMode.telemetry.addLine("Intake: " + intake.getPosition());
    //     myOpMode.telemetry.addLine("Launch Primer: " + launchPrimer.getPosition());
    //     myOpMode.telemetry.addLine("Flipper: " + flipper.getPosition());
    //     myOpMode.telemetry.addLine("Color Sensor: " + colorSensor);
    //     myOpMode.telemetry.addLine("Webcam: " + webcam);
        myOpMode.telemetry.update();
    }
    
    public void driveRobot(double forward, double rotation, double strafe) {
        // Combine drive and turn for blended motion.
        double leftFrontPower  =   forward + strafe + rotation;
        double rightFrontPower = - forward + strafe + rotation;
        double leftBackPower   = - forward - strafe + rotation;
        double rightBackPower  =   forward - strafe + rotation;
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
        motor3.setPower(leftFrontPower  / 3);
        //back right
        motor2.setPower(rightFrontPower / 3);
        //back left
        motor1.setPower(leftBackPower / 3);
        //front left
        motor0.setPower(rightBackPower / 3);
        //front right
    }
    
// ---------- tools End ------- \\

}
