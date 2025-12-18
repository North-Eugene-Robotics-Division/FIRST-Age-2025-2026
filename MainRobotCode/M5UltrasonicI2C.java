package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class M5UltrasonicI2C {
    //commented version: https://github.com/North-Eugene-Robotics-Division/FIRST-Age-2025-2026/commit/7b2f73a89581da9cc19799957a14f5e5b052601e
    // Tune these
    private static int I2C_BUS;              // you confirmed bus 1
    private static final int I2C_ADDR_7BIT = 0x57;
    private static final int READ_DELAY_MS = 200;      // stable for you
    private static final int OFFSET_MM = 1000;         // your calibration (100cm)
    private static final int MIN_UPDATE_MS = 60;       // rate limit

    private final LinearOpMode opMode;
    private LynxI2cDeviceSynchV2 i2c;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean waitingForRead = false;
    private double triggerTimeMs = 0;

    private int lastMm = 0;
    private double lastUpdateMs = 0;

    public M5UltrasonicI2C(LinearOpMode opMode, int bus) {
        this.opMode = opMode;
        this.I2C_BUS = bus;
    }

    public void init() {
        List<LynxModule> modules = opMode.hardwareMap.getAll(LynxModule.class);
        if (modules.isEmpty()) {
            throw new RuntimeException("No LynxModule found (Control Hub not detected).");
        }

        LynxModule module = modules.get(0);

        i2c = new LynxI2cDeviceSynchV2(opMode.hardwareMap.appContext, module, I2C_BUS);
        i2c.setI2cAddress(I2cAddr.create7bit(I2C_ADDR_7BIT));
        i2c.engage();

        timer.reset();
        waitingForRead = false;
        lastMm = 0;
        lastUpdateMs = 0;
    }

    /** Call this once per loop. Non-blocking. */
    public void update() {
        if (i2c == null) return;

        double nowMs = timer.milliseconds();

        // rate limit a bit
        if (!waitingForRead && (nowMs - lastUpdateMs) < MIN_UPDATE_MS) return;

        try {
            if (!waitingForRead) {
                // trigger measurement
                i2c.write(new byte[]{ 0x01 });
                waitingForRead = true;
                triggerTimeMs = nowMs;
            } else {
                // read after delay
                if ((nowMs - triggerTimeMs) >= READ_DELAY_MS) {
                    byte[] b = i2c.read(3);

                    int rawMm = ((b[0] & 0xFF) << 8) | (b[1] & 0xFF);
                    int mm = rawMm - OFFSET_MM;
                    if (mm < 0) mm = 0;

                    lastMm = mm;
                    lastUpdateMs = nowMs;
                    waitingForRead = false;
                }
            }
        } catch (Exception ignored) {
            // If something glitches, just try again next loop
            waitingForRead = false;
        }
    }

    public double getDistanceCm() {
        return lastMm / 10.0;
    }

    public int getDistanceMm() {
        return lastMm;
    }

    public void close() {
        if (i2c != null) i2c.close();
        i2c = null;
    }
}
