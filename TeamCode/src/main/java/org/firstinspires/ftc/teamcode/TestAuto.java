package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@Autonomous
public final class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
            while (opModeInInit() && !isStopRequested()) {
                drive.updatePoseEstimate();
            Pose2d origpose = drive.pose;
            while (opModeInInit()) {
                drive.updatePoseEstimate();
            }
            waitForStart();
            drive.updatePoseEstimate();
            Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(17, -55), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(35, -25), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(45, -25), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(45, -60), Math.toRadians(-90))
                    .build());

        }
    }
}