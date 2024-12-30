package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test_servo", group = "Test")
public class Test_servo extends LinearOpMode{
    //變數設定

    Servo Servo;
    double ServoTargetPosition = 0;
    int max_angle = 180;

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        //初始狀態設定，例如Servo初始位置
        while(opModeIsActive()) {
            //迴圈執行內容

            if(gamepad1.a) {
                ServoTargetPosition += 0.01;
            }
            if(gamepad1.b) {
                ServoTargetPosition -= 0.01;
            }
            Servo.setPosition(ServoTargetPosition);

            telemetry.addData("ServoTargetPosition", ServoTargetPosition);
            telemetry.addData("Servo.getPosition()", Servo.getPosition());
            telemetry.addData("Servo angle", Servo.getPosition() * max_angle);
            telemetry.update();
            idle();
        }
    }
    // 建立函式
    private void init_hardware() {
        //設定物件

        Servo = hardwareMap.get(Servo.class, "Servo");

        idle();
    }
}
// 外面不可以寫程式喔!!!