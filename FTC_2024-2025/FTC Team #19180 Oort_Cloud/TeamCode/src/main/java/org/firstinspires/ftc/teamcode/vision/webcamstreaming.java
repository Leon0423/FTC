package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Vision")
public class webcamstreaming extends LinearOpMode{
    //變數設定

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();

        //設定物件

        // Initializing a Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // To create an OpenCvWebcam
        //TODO: change your webcam name as the DS setting
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // With live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Opening the Camera Device
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /* Usually this is where you'll want to start streaming from the camera (see section 4)
                 * Set your preferred resolution
                 * 320x240 320 像素 240 英寸
                 * 640x480 640 像素 480 像素
                 * 1280x720 1280 像素 720 像素
                 * 1920x1080 1920 像素 1080 像素
                 */
                camera.startStreaming(640, 480);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("Error", "Camera failed to open");
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        idle();

        waitForStart();
        //初始狀態設定，例如Servo初始位置
        while(opModeIsActive()) {
            //迴圈執行內容



            idle();
        }
        camera.stopStreaming();
    }
    // 建立函式
    private void init_hardware() {

    }

    class EmptyPipeline extends OpenCvPipeline
    {
        @Override
        public Mat processFrame(Mat input)
        {
            return input;
        }
    }
}
// 外面不可以寫程式喔!!!