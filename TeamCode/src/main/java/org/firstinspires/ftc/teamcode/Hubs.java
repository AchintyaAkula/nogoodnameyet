package org.firstinspires.ftc.teamcode;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;
import java.util.Arrays;

@SuppressWarnings("unused")
public class Hubs {
    private static final Logger log = LoggerFactory.getLogger(Hubs.class);
    private final ByteBuffer data;
    private final IntBuffer intData;

    private int motorUsages = 0b00000000;
    private int servoUsages = 0b000000000000;

    private boolean updateNeeded = false;
    private final int[] motorPowers = new int[8];
    private final int[] servoPositions = new int[12];

    private final boolean[] digitalChannelValues = new boolean[16];
    private final int[] analogEncoderValues = new int[8];
    private final int[] encoderPositionValues = new int[8];
    private final int[] encoderVelocityValues = new int[8];

    public static Hubs INSTANCE = new Hubs();

    // Constructor
    private Hubs() {
        System.loadLibrary("rust_sdk");
        data = ByteBuffer.allocateDirect(100).order(ByteOrder.nativeOrder());
        intData = data.asIntBuffer();
    }

    // Native rust functions
    private native void initializeRustPipeline(int motorUsages, int ServoUsages, int hubCount);
    private native void shutdownRustPipeline();
    private native void internalUpdate(
            int cm0, int cm1, int cm2, int cm3,
            int em0, int em1, int em2, int em3,
            int cs0, int cs1, int cs2, int cs3, int cs4, int cs5,
            int es0, int es1, int es2, int es3, int es4, int es5
    );
    private native void getAllData(ByteBuffer buffer);

    // All hardware registry functions

    // Register a motor as being used
    public void registerMotor(int portNum, boolean controlHub) {
        int port = controlHub ? portNum : portNum + 4;
        motorUsages |= (1 << port);
    }
    // Register a servo as being used
    public void registerServo(int portNum, boolean controlHub) {
        int port = controlHub ? portNum : portNum + 6;
        servoUsages |= (1 << port);
    }

    // All hardware read functions

    // Return digital true/false value
    public boolean getDigitalValue( int portNum, boolean controlHub) {
        int port = controlHub ? portNum : portNum + 8;
        return digitalChannelValues[port];
    }
    // Return analog value
    public int getAnalogValue(int portNum, boolean controlHub) {
        int port = controlHub ? portNum : portNum + 4;
        return analogEncoderValues[port];
    }
    // Return motor encoder position
    public int getEncPosition(int portNum, boolean controlHub) {
        int port = controlHub ? portNum : portNum + 4;
        return encoderPositionValues[port];
    }
    // Return motor encoder velocity
    public int getEncVelocity(int portNum, boolean controlHub) {
        int port = controlHub ? portNum : portNum + 4;
        return encoderVelocityValues[port];
    }

    // All hardware write functions

    // Set Power to the specified motor
    public void setPower(int portNum, boolean controlHub, double power) {
        int port = controlHub ? portNum : portNum + 4;
        int powerInt = (int) (power * 32767);
        if (motorPowers[port] != powerInt) {
            log.info("I just set a power from port" + port);
            motorPowers[port] = powerInt;
            updateNeeded = true;
        }
    }
    // Set Position to the specified servo
    public void setPosition(int portNum, boolean controlHub, int position) {
        int port = controlHub ? portNum : portNum + 6;
        if (servoPositions[port] != position) {
            servoPositions[port] = position;
            updateNeeded = true;
        }
    }

    // Initialize the rust pipeline
    public void initialize(int hubCount) {
        initializeRustPipeline(motorUsages, servoUsages, hubCount);
    }
    // Update and send the necessary data over to rust, and get bulk data
    public void update() {
        updateActuatorWrites();
        bulkReadData();
    }
    // Send all writes as necessary
    private void updateActuatorWrites() {
        if (updateNeeded) {
            internalUpdate(
                    motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3],
                    motorPowers[4], motorPowers[5], motorPowers[6], motorPowers[7],
                    servoPositions[0], servoPositions[1], servoPositions[2], servoPositions[3], servoPositions[4], servoPositions[5],
                    servoPositions[6], servoPositions[7], servoPositions[8], servoPositions[9], servoPositions[10], servoPositions[11]
            );
            updateNeeded = false;
        }
    }
    // Process returned bulk data into lists
    private void bulkReadData() {
        getAllData(data);
        intData.position(0);
        intData.get(encoderPositionValues, 0, 8);
        intData.get(encoderVelocityValues, 0, 8);
        intData.get(analogEncoderValues, 0, 8);
        int digital = intData.get(24);
        int cHub = digital & 0xFF;
        int eHub = (digital >> 8) & 0xFF;
        for (int i = 0; i <= 7; i++) {
            digitalChannelValues[i] = (cHub & (1 << i)) != 0;
        }
        for (int i = 0; i <= 7; i++) {
            digitalChannelValues[i + 8] = (eHub & (1 << i)) != 0;
        }
    }
    // Shutdown the rust pipeline and reset everything
    public void shutDown() {
        shutdownRustPipeline();
        intData.position(0);
        int[] empty = new int[25];
        intData.put(empty);
        motorUsages = 0;
        servoUsages = 0;
        Arrays.fill(motorPowers, 0);
        Arrays.fill(servoPositions, 0);
        Arrays.fill(digitalChannelValues, false);
        Arrays.fill(analogEncoderValues, 0);
        Arrays.fill(encoderPositionValues, 0);
        Arrays.fill(encoderVelocityValues, 0);
    }
}