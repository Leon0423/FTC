package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class SampleVisionOpMode extends LinearOpMode {

    private CSVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        visionProcessor = new CSVisionProcessor(50, 0, 0, 0, 0, 0, 0); //use this contstructor to specify the rectange size and position
        //i.e. visionProcessor = new CSVisionProcessor(100, 100, 50, 150, 50, 200, 50);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);

        CSVisionProcessor.StartingPosition startingPos = CSVisionProcessor.StartingPosition.NONE;

        //waitForStart();

        while(!this.isStarted() && !this.isStopRequested()) {
            startingPos = visionProcessor.getStartingPosition();
            telemetry.addData("Identified", visionProcessor.getStartingPosition());
            telemetry.update();
        }

        visionPortal.stopStreaming();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //use the value of startingPos to determine your location
            if (CSVisionProcessor.getIntPosition() == 1){
                telemetry.addData("1 - LEFT", startingPos);
            }
            if (CSVisionProcessor.getIntPosition() == 2){
                telemetry.addData("2 - CENTER", startingPos);
            }
            if (CSVisionProcessor.getIntPosition() == 3){
                telemetry.addData("3 - RIGHT", startingPos);
            }
        }

    }






}
