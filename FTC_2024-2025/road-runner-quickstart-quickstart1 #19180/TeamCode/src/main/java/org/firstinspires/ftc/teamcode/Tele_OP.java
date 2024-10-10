package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Tele_OP {
    public class samplePipeline1 extends OpenCvPipeline {

        private String output = "nothing";

        public samplePipeline1() {

        }

        // Mat is the image matrix that should be processed.
        @Override
        public Mat processFrame(Mat input) {
            output = "Sample Pipeline1 Is Running!";
            return input;
        }

        public String getOutput() {
            return output;
        }
    }

    public class samplePipeline2 extends OpenCvPipeline {

        private String output = "nothing";

        public samplePipeline2() {

        }

        // Mat is the image matrix that should be processed.
        @Override
        public Mat processFrame(Mat input) {
            output = "Sample Pipeline2 Is Running!";
            return input;
        }

        public String getOutput() { return output; }
    }

    public class Camera {
        private OpenCvWebcam webcam;
        private HardwareMap hardwareMap;
        private samplePipeline1 p1; // sample pipeline
        private samplePipeline2 p2; // another sample pipeline

        public Camera(HardwareMap hw) { // hardware map from the base class is a parameter
            p1 = new samplePipeline1(); // initialize your pipeline classes
            p2 = new samplePipeline2();

            this.hardwareMap = hw;    //Configure the Camera in hardwaremap
            int cameraMonitorViewId =
                    hardwareMap
                            .appContext
                            .getResources()
                            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            // Get camera from hardware map, replace 'camera' with what is in your controlhub
            webcam =
                    OpenCvCameraFactory.getInstance()
                            .createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

            webcam.setPipeline(p1); // Setting the intial pipeline

            webcam.setMillisecondsPermissionTimeout(2500);

            // Streaming Frames
            webcam.openCameraDeviceAsync(
                    new OpenCvCamera.AsyncCameraOpenListener() {
                        @Override
                        public void onOpened() {
                            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                        }

                        @Override
                        public void onError(int errorCode) {}
                    });
        }

        // Switching Between Pipelines
        public void switchToSecondPipeline(){
            webcam.setPipeline(p2);
        }

        public void switchToFirstPipeline(){
            webcam.setPipeline(p1);
        }

        // Get information from pipeline
        public String getPipeline1Output(){
            return p1.getOutput();
        }

        // call stop at the end of the opMode.
        public void stop() {
            webcam.stopStreaming();
        }
    }

    @TeleOp(name = "Sample_TeleOP", group = "robot")
    public class Sameple_TeleOP extends LinearOpMode {
        Camera camera = new Camera(hardwareMap);

        @Override
        public void runOpMode() throws InterruptedException {
            camera.switchToFirstPipeline();
            telemetry.addLine("Status: Initialized");
            waitForStart();

            while (opModeIsActive()) {
                // OpMode receives the information transmitted from the pipeline class
                // to the camera module class.
                telemetry.addLine(camera.getPipeline1Output());
                telemetry.update();
            }
        }
    }
}
