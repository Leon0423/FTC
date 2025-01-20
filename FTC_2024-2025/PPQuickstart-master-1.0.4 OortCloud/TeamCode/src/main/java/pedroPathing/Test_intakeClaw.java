package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test_intakeClaw", group = "Test")
public class Test_intakeClaw extends LinearOpMode{
    //變數設定

    Servo servoLeft, servoRight, claw;
    double servo_pos = 0;
    double clawMaximum = 0.5;
    double clawMinimum = 0.3;

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {
            // * 迴圈執行內容
            servo_pos = Math.min(1, Math.max(0, servo_pos));
            servo_pos += -gamepad1.right_stick_y * 0.01;

            servoLeft.setPosition(servo_pos);
            servoRight.setPosition(servo_pos);

            if(gamepad1.x) {
                claw.setPosition(clawMaximum);
            }
            if(gamepad1.y) {
                claw.setPosition(clawMinimum);
            }

            telemetry.addData("servoRight", servoRight.getPosition());
            telemetry.addData("servoLeft", servoLeft.getPosition());
            telemetry.addData("clawPosition", claw.getPosition());

            telemetry.update();
            idle();
        }
    }
    // 建立函式
    private void init_hardware() {
        //設定物件
        //初始狀態設定，例如Servo初始位置
        servoLeft = hardwareMap.get(Servo.class, "intakeLeft");
        servoRight = hardwareMap.get(Servo.class, "intakeRight");
        claw = hardwareMap.get(Servo.class, "claw");
        servoRight.setDirection(Servo.Direction.REVERSE);
        servoRight.setPosition(servo_pos);
        servoLeft.setPosition(servo_pos);
        idle();
    }
}
// 外面不可以寫程式喔!!!