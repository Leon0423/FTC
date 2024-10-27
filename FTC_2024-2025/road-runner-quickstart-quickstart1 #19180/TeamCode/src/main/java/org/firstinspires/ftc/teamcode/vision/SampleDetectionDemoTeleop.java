package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(name = "Sample Detection Demo Teleop")
public class SampleDetectionDemoTeleop extends LinearOpMode {
    OpenCvCamera webcam;
    RedBlueDetectionPipelineNoPNP pipeline;


    @Override
    public void runOpMode() {
        // Get the camera monitor view ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new RedBlueDetectionPipelineNoPNP();
        webcam.setPipeline(pipeline);

        webcam.openCameraDevice();
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480); // Set your preferred resolution
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to open");
                telemetry.update();
            }
        });

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // stream on the FTC dashboard
        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ArrayList<RedBlueDetectionPipelineNoPNP.AnalyzedStone> objectCount = pipeline.getDetectedStones();
            telemetry.addData("Objects Detected", objectCount);
            telemetry.update();
        }
        webcam.stopStreaming();
    }
}