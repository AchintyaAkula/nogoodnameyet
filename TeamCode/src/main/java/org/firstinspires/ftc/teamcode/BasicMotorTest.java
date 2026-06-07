package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Motor Test", group = "Tests")
public class BasicMotorTest extends OpMode {

    // I just added this for quick access, change this to match ur actual port
    // It will be either 0,1,2,3
    public int motorPortNum = 0;
    // Is it on the control hub?
    public boolean isOnControlHub = true;

    @Override
    public void init() {
        Hubs.INSTANCE.initialize();
        // Enter in whether it is on the control hub or not with a true/false value, and the port number, either 0,1,2,3
        Hubs.INSTANCE.registerMotor(motorPortNum, isOnControlHub);
    }

    @Override
    public void loop() {
        Hubs.INSTANCE.setPower(motorPortNum, isOnControlHub, 1.0);
        Hubs.INSTANCE.update();
        telemetry.addData("position", Hubs.INSTANCE.getEncPosition(motorPortNum, isOnControlHub));
        telemetry.addData("velocity", Hubs.INSTANCE.getEncVelocity(motorPortNum, isOnControlHub));
        telemetry.update();
    }
}
