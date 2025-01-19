package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Test_Arm", group = "Test")
public class Test_Arm extends LinearOpMode{
    //變數設定

    private CRServo ArmLeft, ArmRight;
    private Servo OutputClaw;
    private double ArmPower = 0.5;
    ElapsedTime rotateTimer = new ElapsedTime();
    private double ClawRotation_TimeLimit = 1.0;

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {
            // * 迴圈執行內容
            if (gamepad2.a) {
                ArmLeft.setPower(ArmPower);
                ArmRight.setPower(ArmPower);
            } else if (gamepad2.b) {
                ArmLeft.setPower(-ArmPower);
                ArmRight.setPower(-ArmPower);
            }

            if (gamepad2.dpad_up) {
                rotateTimer.reset();
                while(rotateTimer.time(TimeUnit.SECONDS) < ClawRotation_TimeLimit) {
                    ArmLeft.setPower(ArmPower);
                    ArmRight.setPower(-ArmPower);
                }
            } else if (gamepad2.dpad_down) {
                rotateTimer.reset();
                while(rotateTimer.time(TimeUnit.SECONDS) < ClawRotation_TimeLimit) {
                    ArmLeft.setPower(-ArmPower);
                    ArmRight.setPower(ArmPower);
                }
            }

            ArmLeft.setPower(0.00001);
            ArmRight.setPower(0.00001);

            if(gamepad2.x) {
                OutputClaw.setPosition(OutputClaw.getPosition() + 0.001);
            }
            if(gamepad2.y) {
                OutputClaw.setPosition(OutputClaw.getPosition() - 0.001);
            }


            telemetry.addData("ArmPower", ArmPower);
            telemetry.addLine("Gamepad2 A / B : Arm Up / Down");
            telemetry.addLine("Gamepad2 Dpad Up  /Down : Claw Rotation");
            telemetry.addData("ArmLeftPower", ArmLeft.getPower());
            telemetry.addData("ArmRightPower", ArmRight.getPower());
            telemetry.addLine("Gamepad2 X / Y : Claw Open / Close");
            telemetry.addData("OutputClawPosition", OutputClaw.getPosition());
            telemetry.update();
            idle();
        }
    }
    // 建立函式
    private void init_hardware() {
        //設定物件
        //初始狀態設定，例如Servo初始位置
        ArmLeft = hardwareMap.get(CRServo.class, "ArmLeft");
        ArmRight = hardwareMap.get(CRServo.class, "ArmRight");
        ArmRight.setDirection(CRServo.Direction.REVERSE);

        OutputClaw = hardwareMap.get(Servo.class, "OutputClaw");


        idle();
    }
}
// 外面不可以寫程式喔!!!