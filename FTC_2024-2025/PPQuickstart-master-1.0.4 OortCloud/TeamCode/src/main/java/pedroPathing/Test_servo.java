package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test_horizonSlide", group = "Test")
public class Test_servo extends LinearOpMode{
    //變數設定

    Servo servoLeft, servoRight;
    int max_angle = 180;
    double position = 0;

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        //初始狀態設定，例如Servo初始位置
        while(opModeIsActive()) {
            //迴圈執行內容
            position = Math.min(0.4, Math.max(0, position));
            position += (-gamepad1.left_stick_y) * 0.001;

            //Maximum angle is 0.4
            servoLeft.setPosition(position);
            servoRight.setPosition(position);

            telemetry.addData("servoRight", servoRight.getPosition());
            telemetry.addData("servoLeft", servoLeft.getPosition());
            telemetry.addData("Servo angle", servoLeft.getPosition() * max_angle);
            telemetry.update();
            idle();
        }
    }
    // 建立函式
    private void init_hardware() {
        //設定物件

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoRight.setDirection(Servo.Direction.REVERSE);

        idle();
    }
}
// 外面不可以寫程式喔!!!