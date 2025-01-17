package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test_motor", group = "Test")
public class Test_motor extends LinearOpMode{
    //變數設定

    DcMotor motor;
    int motorTargetPosition = 0;

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {
            //迴圈執行內容

            if(gamepad1.a && motorTargetPosition < 1000) {
                motorTargetPosition += 5;
            }
            if(gamepad1.b && motorTargetPosition > 0) {
                motorTargetPosition -= 5;
            }
            motor.setTargetPosition(motorTargetPosition);

            telemetry.addData("motorTargetPosition", motorTargetPosition);
            telemetry.update();
            idle();
        }
    }
    // 建立函式
    private void init_hardware() {
        //設定物件
        //初始狀態設定，例如Servo初始位置
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(0);
        motor.setPower(0.5);


        idle();
    }
}
// 外面不可以寫程式喔!!!