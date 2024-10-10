package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.RotatedRect;

@Autonomous
public class SampleDetect extends LinearOpMode {

    @Override
    public void runOpMode() {
        BlockDetectorProcessor processor = new BlockDetectorProcessor(telemetry);
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            RotatedRect block = processor.getClosestBlock();
            Double angle = processor.getAngle();

            // Use block and angle information for robot control

            sleep(20);
        }
    }
}