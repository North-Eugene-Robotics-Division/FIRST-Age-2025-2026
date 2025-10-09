package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Hardware {
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // Declare OpMode members.
    public LinearOpMode myOpMode = null;

    public DigitalChannel frontDistance;
    public static double speedOfSound = 0.0343;
    
    public void init() {
        frontDistance = myOpMode.hardwareMap.get(DigitalChannel.class, "DistanceSensor");
    }
}
