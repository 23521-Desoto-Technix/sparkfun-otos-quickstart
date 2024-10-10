package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class ServoTest extends OpMode {
    ServoImplEx claw;
    ServoImplEx wrist;
    AnalogInput analogInput;
    @Override
    public void init() {
        claw = (ServoImplEx) hardwareMap.get(Servo.class, "claw");
        wrist = (ServoImplEx) hardwareMap.get(Servo.class, "wrist");
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        claw.setPosition(0.5);
        wrist.setPosition(0.5);
        analogInput = hardwareMap.get(AnalogInput.class, "wristanalog");
    }

    @Override
    public void loop() {
        claw.setPosition((gamepad1.left_stick_y/2) + 0.5);
        wrist.setPosition((gamepad1.right_stick_y/2) + 0.5);
        telemetry.addData("Claw", (gamepad1.left_stick_y/2) + 0.5);
        telemetry.addData("Wrist", (gamepad1.right_stick_y/2) + 0.5);
        double wristanalog = analogInput.getVoltage() / 3.3;
        telemetry.addData("Analog", wristanalog * 360);

    }
}
