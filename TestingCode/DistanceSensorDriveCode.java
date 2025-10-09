package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name="Drive_Code", group="Linear OpMode")

public class DriveCode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        
        Hardware robot = new Hardware(this);
        robot.init();
    
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
          robot.frontDistance.setMode(DigitalChannel.Mode.OUTPUT);
          robot.frontDistance.setState(false); // Low for 2 microseconds
          sleep(0, 2, robot); // delayMicroseconds(2)
          robot.frontDistance.setState(true);  // High for 5 microseconds
          sleep(0, 5, robot); // delayMicroseconds(5)
          robot.frontDistance.setState(false); // Low to end pulse

           // Change to input mode to read echo
           robot.frontDistance.setMode(DigitalChannel.Mode.INPUT);

           long startTime = System.nanoTime();
           long endTime = startTime;
           long pulseDuration = 0;

           // Wait for the echo pulse to go HIGH
           while (!robot.frontDistance.getState() && (System.nanoTime() - startTime) < 50000000L) { // Timeout after 50ms
               // Wait
           }
           startTime = System.nanoTime(); // Reset start time for pulse duration measurement

           // Wait for the echo pulse to go LOW
           while (robot.frontDistance.getState() && (System.nanoTime() - startTime) < 50000000L) { // Timeout after 50ms
               // Wait
           }
           
           endTime = System.nanoTime();

           pulseDuration = (endTime - startTime) / 1000; // Convert nanoseconds to microseconds

           // Calculate distance in centimeters
            double distanceCM = (pulseDuration * robot.speedOfSound) / 2.0; // Divide by 2 because it's round trip
            
            telemetry.addData("NanoTime", System.nanoTime());
            telemetry.addData("Pulse Duration (us)", pulseDuration);
            telemetry.addData("Distance (cm)", "%.2f", distanceCM);
            telemetry.update();

           sleep(100, 0, robot); // Small delay before next reading
       }
   }

   // Custom sleep function for microsecond delays
  private void sleep(long millis, int nanos, Hardware robot) {
      try {
        Thread.sleep(millis, nanos);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
  }
}
