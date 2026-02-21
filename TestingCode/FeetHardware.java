package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class FeetHardware {
    private LinearOpMode myOpMode = null;
    public DcMotor motor = null;
    
    public FeetHardware (LinearOpMode opMode) {
        myOpMode = opMode;
    }
    
    public void init(){
        motor = myOpMode.hardwareMap.get(DcMotor.class, "motor");
        
        //Sets encoder to 0 at start
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Set motor to run without encoders even though we have some
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }
    
    int FeetPosition = 3000;
    boolean isMotorBusy = false;
    
    public void moveFeet(){
        if (motor.getCurrentPosition() <= FeetPosition) {
            motor.setPower(1);
            isMotorBusy = true;
        }
        
        if (motor.getCurrentPosition() > FeetPosition) {
            motor.setPower(-1);
            isMotorBusy = true;
        }
    }
}
