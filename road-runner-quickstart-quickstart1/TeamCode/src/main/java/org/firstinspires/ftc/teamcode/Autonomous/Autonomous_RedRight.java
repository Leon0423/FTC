package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous_Base;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Autonomous_RedRight")

public class Autonomous_RedRight extends Autonomous_Base {

    //slide
    private DcMotorEx slide_left = null, slide_right = null;
    private int slidepos =800;
    private double slide_speed = 0.75;

    //intake
    private DcMotorEx intake = null;

    //claw
    private Servo clawServo = null;
    // 伺服馬達的初始位置
    private double clawPosition = 0.5;

    //arm
    private Servo armServo = null;
    private double forwardPosition = 0.92; // 正轉位置

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    boolean LEFTmode = false, CENTERmode = false, RIGHTmode = false;

    @Override
    public void runOpMode() {

        init_hardware();
        clawServo.setPosition(0.65);
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        TrajectorySequence Lefttrail = drive.trajectorySequenceBuilder(new Pose2d(9.50, -63.00, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(1.70,() -> {
                    intake.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.20,() -> {
                    intake.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.70,() -> {
                    slide_left.setTargetPosition(slidepos);
                    slide_right.setTargetPosition(slidepos);
                    slide_right.setPower(slide_speed);
                    slide_left.setPower(slide_speed);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.80,() -> {
                    armServo.setPosition(forwardPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(6.00,() -> {
                    clawPosition = 0.5;
                    clawServo.setPosition(clawPosition);
                    slide_left.setTargetPosition(0);
                    slide_right.setTargetPosition(0);
                })
                .splineTo(new Vector2d(6.50, -37.50), Math.toRadians(135.00))
                .lineToSplineHeading(new Pose2d(51.50, -37.50, Math.toRadians(180.00)))
                .waitSeconds(2.2)
                .lineTo(new Vector2d(51.50, -63.00))
                .build();



        TrajectorySequence Centertrail = drive.trajectorySequenceBuilder(new Pose2d(9.50, -63.00, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(1.83,() -> {
                    intake.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.20,() -> {
                    intake.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.84,() -> {
                    slide_left.setTargetPosition( slidepos );
                    slide_right.setTargetPosition( slidepos );
                    slide_right.setPower(slide_speed);
                    slide_left.setPower(slide_speed);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.50,() -> {
                    armServo.setPosition(forwardPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(6.00,() -> {
                    clawPosition = 0.5;
                    clawServo.setPosition(clawPosition);
                    slide_left.setTargetPosition(0);
                    slide_right.setTargetPosition(0);
                })
                .lineToConstantHeading(new Vector2d(9.50, -35.00))
                .lineToSplineHeading(new Pose2d(51.50, -35.00, Math.toRadians(180.00)))
                .waitSeconds(2.2)
                .lineTo(new Vector2d(51.50, -63.00))
                .build();


        TrajectorySequence Righttrail = drive.trajectorySequenceBuilder(new Pose2d(9.50, -63.00, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(1.75,() -> {
                    intake.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.00,() -> {
                    intake.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.70,() -> {
                    slide_left.setTargetPosition( slidepos );
                    slide_right.setTargetPosition( slidepos );
                    slide_right.setPower(slide_speed);
                    slide_left.setPower(slide_speed);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.00,() -> {
                    armServo.setPosition(forwardPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.10,() -> {
                    clawPosition = 0.5;
                    clawServo.setPosition(clawPosition);
                    slide_left.setTargetPosition(0);
                    slide_right.setTargetPosition(0);
                })
                .lineToConstantHeading(new Vector2d(23.00, -40.00))
                .lineToSplineHeading(new Pose2d(51.50, -40.00, Math.toRadians(180.00)))
                .waitSeconds(2.2)
                .lineTo(new Vector2d(51.50, -63.00))
                .build();


        while(!opModeIsActive()){

            telemetry.addData("LEFT", LEFTmode);
            telemetry.addData("CENTER", CENTERmode);
            telemetry.addData("RIGHT", RIGHTmode);

            if (cX > 0 && cX < 150){
                LEFTmode = true;
                CENTERmode = false;
                RIGHTmode = false;
            } else if (cX < 450) {
                LEFTmode = false;
                CENTERmode = true;
                RIGHTmode = false;
            } else if(cX < 700){
                LEFTmode = false;
                CENTERmode = false;
                RIGHTmode = true;
            }

            telemetry.addData("cX", cX);
            telemetry.update();
        }

        while (opModeIsActive()) {

            if(LEFTmode){
                drive.setPoseEstimate(Lefttrail.start());
                drive.followTrajectorySequence(Lefttrail);
                break;

            }
            if (CENTERmode){
                drive.setPoseEstimate(Centertrail.start());
                drive.followTrajectorySequence(Centertrail);
                break;
            }
            if (RIGHTmode){
                drive.setPoseEstimate(Righttrail.start());
                drive.followTrajectorySequence(Righttrail);
                break;
            }

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(107, 110, 3);
            Scalar upperYellow = new Scalar(150, 255, 145);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }


}
