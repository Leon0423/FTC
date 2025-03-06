package pedroPathing.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(name = "Sample Detection Demo Teleop")
public class SampleDetectionDemoTeleop extends LinearOpMode {
    private static final int MIN_ANGLE = 1;
    private static final int MAX_ANGLE = 180;

    private double rotation = 0; // Rotation variable to keep track of the angle (0-360)
    private double previousAdjustedAngle = 0; // To track the previous value of adjustedAngle

    private Servo servo;
    OpenCvCamera webcam;
    SampleDetectionPipelinePNP pipeline;


    @Override
    public void runOpMode() {

        // ! Initialize the servo
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.REVERSE);

        // Get the camera monitor view ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SampleDetectionPipelinePNP();
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
        
        //以下純手工

        while (opModeIsActive()) {

            if(gamepad1.a) {
                webcam.startStreaming(640, 480); // Set your preferred resolution
            } else if(gamepad1.b) {
                webcam.stopStreaming();
            }


            double adjustedAngle = 0;

            ArrayList<SampleDetectionPipelinePNP.AnalyzedStone> stones = pipeline.getDetectedStones();
            telemetry.addData("Number of stones", stones.size());
            if (!stones.isEmpty()) {
                SampleDetectionPipelinePNP.AnalyzedStone largestStone = stones.get(0);
                for (SampleDetectionPipelinePNP.AnalyzedStone stone : stones) {
                    if (stone.area > largestStone.area) largestStone = stone;
                }
                adjustedAngle = ( largestStone.angle - 180.0) * -1;
                telemetry.addData("DetectionAngle", adjustedAngle);
                telemetry.addData("largestStoneColor", largestStone.color);
            }

            // ! Update the rotation based on the adjusted angle
            updateRotation(adjustedAngle);
            telemetry.addData("Rotation", rotation);

            // ! Set the servo position based on the rotation
            if (getRotation() > 330 && getRotation() < 360) {
                servo.setPosition((getRotation() - 180) / 327);
            } else {
                servo.setPosition(getRotation() / 327);
            }

            telemetry.update();
        }
        webcam.stopStreaming();
    }


    public void updateRotation(double adjustedAngle) {
        // Determine if the rotation is clockwise or counterclockwise
        double angleDifference = adjustedAngle - previousAdjustedAngle;

        // Handle wrap-around at 180 to 1 or 1 to 180
        if (angleDifference > 90) { // Jump from 180 to 1 (clockwise)
            angleDifference -= 180;
        } else if (angleDifference < -90) { // Jump from 1 to 180 (counterclockwise)
            angleDifference += 180;
        }

        // Update the rotation
        rotation += angleDifference;

        // Ensure rotation stays within 0-360 range
        if (rotation < 0) {
            rotation += 360;
        } else if (rotation >= 360) {
            rotation -= 360;
        }

        // Update the previous angle
        previousAdjustedAngle = adjustedAngle;
    }

    public double getRotation() {
        return rotation;
    }


}
