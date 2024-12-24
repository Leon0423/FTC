package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Axon_Mode")
public class Axon_Mode extends LinearOpMode{
    //變數設定
    Servo servo;
    double position = 0.0;
    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        //初始狀態設定，例如Servo初始位置
        while(opModeIsActive()) {
            //迴圈執行內容
            //例如：控制伺服馬達轉動
            if(gamepad1.right_stick_x != 0) {
                position += gamepad1.right_stick_x * 0.001;
            }
            position = Math.min(1, Math.max(0, position));
            servo.setPosition(position);


            telemetry.addData("position", position * 325);
            telemetry.update();

            idle();
        }
    }
    // 建立函式
    private void init_hardware() {
        //設定物件

        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(position);

        idle();
    }
}
// 外面不可以寫程式喔!!!
