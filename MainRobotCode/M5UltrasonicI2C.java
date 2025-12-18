package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * M5UltrasonicI2C
 *
 * This class talks to the M5Stack Ultrasonic I2C unit (RCWL-9620-based) directly
 * using the Control Hub's I2C bus.
 *
 * IMPORTANT CONCEPT:
 * - We do NOT use the robot configuration "device type" dropdown for this sensor.
 *   The Control Hub doesn't have a built-in driver for it.
 * - Instead, we use LynxI2cDeviceSynchV2 to manually write/read bytes over I2C.
 *
 * HOW YOU USE THIS CLASS:
 * 1) Create it (pass your OpMode):   ultrasonic = new M5UltrasonicI2C(this);
 * 2) Call init() once:              ultrasonic.init();
 * 3) In your main loop, call:       ultrasonic.update();   <-- every loop
 * 4) Anywhere you need distance:    ultrasonic.getDistanceCm()
 *
 * WHY update() EXISTS:
 * - The sensor needs time between "trigger measurement" and "read result".
 * - We do NOT want to sleep() in your DriveCode loop (that causes lag).
 * - So we use a tiny state machine:
 *      trigger -> wait (no blocking) -> read -> store value
 */
public class M5UltrasonicI2C {

    // ----------------------------
    // Configuration / tuning knobs
    // ----------------------------

    /**
     * Which Control Hub I2C BUS/PORT the sensor is plugged into.
     * You tested and confirmed the sensor responds on BUS 1.
     *
     * NOTE: This corresponds to the physical I2C port number on the hub.
     */
    private static final int I2C_BUS = 1;

    /**
     * 7-bit I2C address of the M5 ultrasonic unit.
     * This sensor uses address 0x57.
     */
    private static final int I2C_ADDR_7BIT = 0x57;

    /**
     * Time the sensor needs after triggering (writing 0x01) before the data is ready.
     * You found 200ms is stable on your setup.
     *
     * If this is too small, you'll get inconsistent readings.
     * If this is huge, updates feel slower than necessary.
     */
    private static final int READ_DELAY_MS = 200;

    /**
     * Calibration offset in millimeters.
     *
     * Example:
     * If the sensor tends to read ~100cm too far, that's ~1000mm.
     * Subtracting OFFSET_MM moves the measurements closer to reality.
     *
     * NOTE: Ultrasonics can be influenced by mounting, angles, and reflections,
     * so this is a "practical calibration", not perfect physics.
     */
    private static final int OFFSET_MM = 1000;

    /**
     * Rate limit: how often we allow a new trigger cycle to start.
     * This prevents spamming the I2C bus too fast.
     */
    private static final int MIN_UPDATE_MS = 60;

    // ----------------------------
    // Hardware / runtime references
    // ----------------------------

    /**
     * We store the OpMode so we can access the hardwareMap and appContext.
     */
    private final LinearOpMode opMode;

    /**
     * The actual "raw I2C device" interface to the Control Hub bus.
     * Once engaged, we can write/read bytes.
     */
    private LynxI2cDeviceSynchV2 i2c;

    /**
     * A simple timer that counts milliseconds since init().
     * We use this to implement "wait 200ms" WITHOUT blocking.
     */
    private final ElapsedTime timer = new ElapsedTime();

    // ----------------------------
    // State machine variables
    // ----------------------------

    /**
     * waitingForRead == false means:
     *   - we are ready to trigger a new measurement
     *
     * waitingForRead == true means:
     *   - we already triggered (wrote 0x01)
     *   - we're waiting until READ_DELAY_MS has passed
     *   - then we'll read the bytes
     */
    private boolean waitingForRead = false;

    /**
     * The time (in milliseconds from our timer) when we triggered the measurement.
     * Used to decide when it's safe to read.
     */
    private double triggerTimeMs = 0;

    // ----------------------------
    // Latest measurement storage
    // ----------------------------

    /**
     * Last good distance measurement, in millimeters, AFTER applying OFFSET_MM and clamping.
     * This is what getDistanceMm() and getDistanceCm() return.
     */
    private int lastMm = 0;

    /**
     * The time (ms) when we last updated lastMm.
     * Used for rate limiting.
     */
    private double lastUpdateMs = 0;

    /**
     * Constructor: store the OpMode reference.
     */
    public M5UltrasonicI2C(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * init()
     *
     * Must be called ONCE before update().
     * Finds the Control Hub module, creates the I2C object, sets the address, and engages it.
     */
    public void init() {
        // The Control Hub is a "LynxModule". There should be at least one.
        List<LynxModule> modules = opMode.hardwareMap.getAll(LynxModule.class);
        if (modules.isEmpty()) {
            // If you ever see this, something is extremely wrong (no hub detected by SDK).
            throw new RuntimeException("No LynxModule found (Control Hub not detected).");
        }

        // Typically, the first module is the Control Hub.
        LynxModule module = modules.get(0);

        // Create the I2C device interface on the given bus.
        i2c = new LynxI2cDeviceSynchV2(opMode.hardwareMap.appContext, module, I2C_BUS);

        // Set the I2C address of the sensor (7-bit form).
        i2c.setI2cAddress(I2cAddr.create7bit(I2C_ADDR_7BIT));

        // Engage starts communications.
        i2c.engage();

        // Reset timing/state so we start clean.
        timer.reset();
        waitingForRead = false;
        lastMm = 0;
        lastUpdateMs = 0;
    }

    /**
     * update()
     *
     * Call this once per loop.
     * This method is NON-BLOCKING:
     * - It will either trigger a measurement, OR later read it when enough time passed.
     *
     * The measurement protocol we discovered/used:
     * - Write 0x01   (start measurement)
     * - Wait ~200ms
     * - Read 3 bytes
     * - Distance is in the first two bytes as a 16-bit big-endian value (mm-ish),
     *   and the third byte is ignored (status/checksum/unused).
     */
    public void update() {
        // If init() hasn't happened yet (or close() was called), do nothing.
        if (i2c == null) return;

        // Current time from our timer.
        double nowMs = timer.milliseconds();

        // If we're NOT currently waiting on a read, enforce a minimum time between cycles.
        // This avoids hammering the bus too fast.
        if (!waitingForRead && (nowMs - lastUpdateMs) < MIN_UPDATE_MS) return;

        try {
            if (!waitingForRead) {
                // ----------------------------
                // PHASE 1: Trigger measurement
                // ----------------------------

                // Command byte: 0x01 tells the sensor "start a new measurement".
                i2c.write(new byte[]{ 0x01 });

                // Switch to "waiting" phase.
                waitingForRead = true;

                // Remember when we triggered so we can wait READ_DELAY_MS.
                triggerTimeMs = nowMs;

            } else {
                // ----------------------------
                // PHASE 2: Wait, then read data
                // ----------------------------

                // Only read after enough time has passed.
                if ((nowMs - triggerTimeMs) >= READ_DELAY_MS) {

                    // Read 3 bytes from the sensor.
                    // We ignore b[2] because the useful distance is in b[0], b[1].
                    byte[] b = i2c.read(3);

                    // Combine the first two bytes into a 16-bit number.
                    // big-endian: (highByte << 8) | lowByte
                    int rawMm = ((b[0] & 0xFF) << 8) | (b[1] & 0xFF);

                    // Apply calibration offset.
                    int mm = rawMm - OFFSET_MM;

                    // Clamp: distance should never go negative.
                    if (mm < 0) mm = 0;

                    // Store as latest reading for getters.
                    lastMm = mm;
                    lastUpdateMs = nowMs;

                    // Reset state so next update() can trigger again.
                    waitingForRead = false;
                }
            }
        } catch (Exception ignored) {
            // If anything goes wrong (bus glitch, read failure, etc),
            // don't crash the whole OpMode. Just reset the state and try again next loop.
            waitingForRead = false;
        }
    }

    /**
     * Returns the last stored distance in centimeters.
     * (This is just lastMm converted to cm.)
     */
    public double getDistanceCm() {
        return lastMm / 10.0;
    }

    /**
     * Returns the last stored distance in millimeters.
     */
    public int getDistanceMm() {
        return lastMm;
    }

    /**
     * close()
     *
     * Call this if you want to manually shut down I2C communications
     * (not always necessary in FTC, but good practice for cleanup).
     */
    public void close() {
        if (i2c != null) i2c.close();
        i2c = null;
    }
}
