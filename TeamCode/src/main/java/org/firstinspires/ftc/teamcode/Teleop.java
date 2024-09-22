package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static boolean FIELD_CENTRIC = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_435)
        );

        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU imu = (IMU) hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();
        imu.resetYaw();

        while (!isStopRequested()) {

            // Driving the mecanum base takes 3 joystick parameters: leftX, leftY, rightX.
            // These are related to the left stick x value, left stick y value, and
            // right stick x value respectively. These values are passed in to represent the
            // strafing speed, the forward speed, and the turning speed of the robot frame
            // respectively from [-1, 1].

            if (!FIELD_CENTRIC) {

                // For a robot centric model, the input of (0,1,0) for (leftX, leftY, rightX)
                // will move the robot in the direction of its current heading. Every movement
                // is relative to the frame of the robot itself.
                //
                //                 (0,1,0)
                //                   /
                //                  /
                //           ______/_____
                //          /           /
                //         /           /
                //        /___________/
                //           ____________
                //          /  (0,0,1)  /
                //         /     ↻     /
                //        /___________/

                // optional fourth parameter for squared inputs
                drive.driveRobotCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY(),
                        driverOp.getRightX(),
                        false
                );
            } else {

                // Below is a model for how field centric will drive when given the inputs
                // for (leftX, leftY, rightX). As you can see, for (0,1,0), it will travel forward
                // regardless of the heading. For (1,0,0) it will strafe right (ref to the 0 heading)
                // regardless of the heading.
                //
                //                   heading
                //                     /
                //            (0,1,0) /
                //               |   /
                //               |  /
                //            ___|_/_____
                //          /           /
                //         /           / ---------- (1,0,0)
                //        /__________ /

                // optional fifth parameter for squared inputs
                drive.driveFieldCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY(),
                        driverOp.getRightX(),
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                        false
                );
            }
            if (gamepad1.a) {
                imu.resetYaw();
            }

            if (gamepad1.left_trigger > 0.5) {
                FIELD_CENTRIC = false;
            }
            if (gamepad1.right_trigger > 0.5) {
                FIELD_CENTRIC = true;
            }
            telemetry.addLine("Press A to reset the gyro");
            telemetry.addLine("Press left trigger to switch to robot centric");
            telemetry.addLine("Press right trigger to switch to field centric");
        }
    }

}