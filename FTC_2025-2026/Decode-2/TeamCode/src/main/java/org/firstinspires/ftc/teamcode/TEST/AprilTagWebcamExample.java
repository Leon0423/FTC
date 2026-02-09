package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "AprilTagWebcamExample")
public class AprilTagWebcamExample extends LinearOpMode {
    //變數設定
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagWebcam.init(hardwareMap, telemetry);

        waitForStart();
        //初始狀態設定，例如Servo初始位置
        while(opModeIsActive()) {
            //迴圈執行內容
            aprilTagWebcam.update();
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
            telemetry.addData("id20 String", id20.toString());
            telemetry.update();
            idle();
        }
    }
    // 建立函式

}
// 外面不可以寫程式喔!!!
