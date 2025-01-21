package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test_horizonSlide", group = "Test")
public class Test_servo extends LinearOpMode{
    //變數設定

    Servo Output;

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        //初始狀態設定，例如Servo初始位置
        while(opModeIsActive()) {
            //迴圈執行內容

            if(gamepad1.a) {
                Output.setPosition(1);
            }

            telemetry.update();
            idle();
        }
    }
    // 建立函式
    private void init_hardware() {
        //設定物件

        Output = hardwareMap.get(Servo.class, "OutputClaw");
        //設定初始值
        Output.setPosition(0);


        idle();
    }
}
// 外面不可以寫程式喔!!!