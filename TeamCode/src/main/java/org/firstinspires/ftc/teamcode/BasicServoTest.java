package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Servo Test", group = "Tests")
public class BasicServoTest extends OpMode {

    // I just added this for quick access, change this to match ur actual port
    // It will be either 0,1,2,3,4,5
    public int servoPortNum = 0;
    // Is it on the control hub?
    public boolean isOnControlHub = true;

    @Override
    public void init() {
        Hubs.INSTANCE.registerServo(servoPortNum, isOnControlHub);
        Hubs.INSTANCE.initialize(1);
        // Enter in whether it is on the control hub or not with a true/false value, and the port number, either 0,1,2,3
    }

    @Override
    public void loop() {
        double pos = 0.0;
        Hubs.INSTANCE.setPosition(servoPortNum, isOnControlHub, (int) (500+ (pos * 2000)));
        Hubs.INSTANCE.update();
    }
}
