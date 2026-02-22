package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class FeetHardware {
    private LinearOpMode myOpMode = null;
    public DcMotor Foot = null;
    
    public FeetHardware (LinearOpMode opMode) {
        myOpMode = opMode;
    }
    
    public void init(){
        Foot = myOpMode.hardwareMap.get(DcMotor.class, "motor");
        
        //Sets encoder to 0 at start
        Foot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Set motor to run without encoders even though we have some
        Foot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }
    
    int FeetPosition = 1000;
    boolean isFootBusy = false;
    
    public void moveFeet(){
        if (Foot.getCurrentPosition() <= FeetPosition) {
            Foot.setPower(1);
            isFootBusy = true;
        }
        
        if (Foot.getCurrentPosition() > FeetPosition) {
            Foot.setPower(-1);
            isFootBusy = true;
        }
    }
}
