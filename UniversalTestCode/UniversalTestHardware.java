package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;

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
    private Servo   servo00  = null;
    private Servo   servo01  = null;
    private Servo   servo02  = null;
    private Servo   servo03  = null;
    private Servo   servo04  = null;
    private Servo   servo05  = null;
    private Servo   servo06  = null;
    private Servo   servo07  = null;
    private Servo   servo08  = null;
    private Servo   servo09  = null;
    private Servo   servo10  = null;
    private Servo   servo11  = null;

    public DcMotor[] motors = {};
    
    public String[] motorsNamesList = {"Motor Port 0", "Motor Port 1", "Motor Port 2", "Motor Port 3", "Motor Port 4", "Motor Port 5", "Motor Port 6", "Motor Port 7"};

    public Servo[] servos = {};
    
    public String[] stringsNamesList = {"Servo Port 0", "Motor Port 1", "Motor Port 2", "Motor Port 3", "Motor Port 4", "Motor Port 5", "Motor Port 6", "Motor Port 7"};

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
        
        DcMotor[] motors = {motor0, motor1, motor2, motor3, motor4, motor5, motor6, motor7};
        
        Servo[] servos = {servo00, servo01, servo02, servo03, servo04, servo05, servo06, servo07, servo08, servo09, servo10, servo11};

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
        
        //myOpMode.telemetry.addData(List.toString(motors), List.toString(servos));
        //myOpMode.telemetry.update();
    }

    public int motorNumber = 0;
     
    public String currentMotor = "";
     
    public void cycleMotor() {
        
        motorNumber = (motorNumber + 1)%8;
        myOpMode.telemetry.addData("Active motor", motorsNamesList[motorNumber]);
        myOpMode.telemetry.update();
        
    }
    
    public void motorPowerPos() {
        
        motors[motorNumber].setPower(1);
    }
    
    
    public void motorPowerNeg() {
        
        motors[motorNumber].setPower(-1);
    }

    
}
