package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.BlockDetectorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class SampleAlign extends LinearOpMode {
    ServoImplEx wrist;
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));


        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

    }

    @Override
    public void runOpMode() {
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        BlockDetectorProcessor processor = new BlockDetectorProcessor(telemetry);
        final CameraStreamProcessor streamer = new CameraStreamProcessor();
        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "wristanalog");
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .addProcessor(streamer)
                .build();
        double wristpos= 0.5;
        FtcDashboard.getInstance().startCameraStream(streamer, 0);

        waitForStart();

        while (opModeIsActive()) {
            RotatedRect block = processor.getClosestBlock();
            Double angle = processor.getAngle();
            if (processor.getSize() > 5000) {
                if (90 < angle && angle < 180) {
                    double deltaw = 90 - (angle - 90);
                    deltaw = deltaw / 90 / 20;
                    //telemetry.addData("Delta W", deltaw);
                    wristpos -= deltaw;
                }
                if (0 < angle && angle < 90) {
                    double deltaw = angle / 90 / 20;
                    wristpos += deltaw;
                    //telemetry.addData("Delta W", deltaw);
                }
            }
            wrist.setPosition(wristpos);
            processor.updatetelemetry();

            sleep(20);
        }
    }
}