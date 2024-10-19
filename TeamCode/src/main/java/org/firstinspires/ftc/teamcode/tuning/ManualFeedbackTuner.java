package org.firstinspires.ftc.teamcode.tuning;

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
public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)) {
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
            while (opModeInInit() && !isStopRequested()) {
                drive.updatePoseEstimate();
            }
            Pose2d origpose = drive.pose;
            while (opModeIsActive()) {
                Actions.runBlocking(
                        drive.actionBuilder(origpose)
                                .splineTo(
                                        new Vector2d(
                                                origpose.position.x + 10,
                                                origpose.position.y + 10), Math.toRadians(90))
                                .splineTo(
                                        new Vector2d(
                                                origpose.position.x,
                                                origpose.position.y + 20), Math.toRadians(180))
                                .setReversed(true)
                                .splineTo(
                                        new Vector2d(
                                                origpose.position.x + 10,
                                                origpose.position.y + 10), Math.toRadians(90))
                                .splineTo(
                                        new Vector2d(
                                                origpose.position.x,
                                                origpose.position.y), Math.toRadians(0))
                                .build());
            }
        }
    }
}