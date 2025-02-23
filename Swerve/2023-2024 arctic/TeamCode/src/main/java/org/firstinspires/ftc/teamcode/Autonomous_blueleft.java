package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(name = "Autonomous_blueleft")

public class Autonomous_blueleft extends Autonomous_Base {

    public void purple() {
        SR.setTargetPosition(slideR_targetPos1);
        SL.setTargetPosition(slideL_targetPos1);
        while(Math.abs(SR.getCurrentPosition()-slideR_targetPos1) > 30);
        clawR.setPosition(0.1);
        clawL.setPosition(0.1);
        while(Math.abs(clawR.getPosition() - 0.1) > 0.01);
        sleep(500);
        claw.setPosition(0.975);
        sleep(1000);
        clawR.setPosition(0);
        clawL.setPosition(0);
        sleep(300);
        SR.setTargetPosition(0);
        SL.setTargetPosition(0);
    }

    public void yellow() {
        claw.setPosition(0.85);
        sleep(500);
        SR.setTargetPosition(slideR_targetPos1);
        SL.setTargetPosition(slideL_targetPos1);
        while(Math.abs(SR.getCurrentPosition()-slideR_targetPos1) > 30);
        clawR.setPosition(0.28);
        clawL.setPosition(0.28);
        while(Math.abs(clawR.getPosition() - 0.28) > 0.01);
        sleep(1000);
        claw.setPosition(0.975);
        sleep(500);
        clawR.setPosition(0);
        clawL.setPosition(0);
        sleep(1000);
        SR.setTargetPosition(-10);
        SL.setTargetPosition(-10);
        sleep(500);
    }
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;// Replace with the focal length of the camera in pixels



    int slideR_targetPos1=1050,slideL_targetPos1=1050,slideR_targetPos2=1501,slideL_targetPos2=1501,a=0;
    @Override
    public void runOpMode() {
        init_hardware();
        claw.setPosition(0.85);
        clampingservo.setPosition(0.05);
        clawR.setPosition(0);
        clawL.setPosition(0);
        shooter.setPosition(0.5);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence TrajectoryA1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(36,30,Math.toRadians(180)),Math.toRadians(0))
                .build();

        TrajectorySequence TrajectoryA2 = drive.trajectorySequenceBuilder(TrajectoryA1.end())
                .lineToLinearHeading(new Pose2d(56,40,Math.toRadians(0)))
                .build();

        TrajectorySequence TrajectoryB1 = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .build();

        TrajectorySequence TrajectoryB2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(57,33,Math.toRadians(0)))
                .build();

        TrajectorySequence TrajectoryC1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,28,Math.toRadians(180)))
                .build();

        TrajectorySequence TrajectoryC2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(55,25,Math.toRadians(0)))
                .build();

        TrajectorySequence Trajectorypark = drive.trajectorySequenceBuilder(new Pose2d())
                .back(5)
//                .strafeRight(50)
                .strafeLeft(30)
                .build();


        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        boolean modeA= false ,modeB= false ,modeC= false;


        telemetry.addData("mode", modeA);
        telemetry.addData("mode", modeB);
        telemetry.addData("mode", modeC);
        telemetry.addData("cX", cX);
        telemetry.update();
        sleep(1000);


        waitForStart();
        if(cX < 150) {
            modeA = true;
        }
        else if(cX <450){
            modeB = true;
        }
        else if(cX <700){
            modeC = true;
        }

        if(modeA){
            if(isStopRequested()) return;
            drive.followTrajectorySequence(TrajectoryA1);
            purple();
            drive.followTrajectorySequence(TrajectoryA2);
            yellow();
//            drive.followTrajectorySequence(Trajectorypark);

        }
        if(modeB){
            if(isStopRequested()) return;
            drive.followTrajectorySequence(TrajectoryB1);
            purple();
            drive.followTrajectorySequence(TrajectoryB2);
            yellow();
//            drive.followTrajectorySequence(Trajectorypark);
}
        if(modeC){
            if(isStopRequested()) return;
            drive.followTrajectorySequence(TrajectoryC1);
            purple();
            drive.followTrajectorySequence(TrajectoryC2);
            yellow();
//            drive.followTrajectorySequence(Trajectorypark);
        }
        telemetry.addData("mode", modeA);
        telemetry.addData("mode", modeB);
        telemetry.addData("mode", modeC);
        telemetry.addData("cX", cX);




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
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

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