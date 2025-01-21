package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.Target;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Test_Arm", group = "Test")
public class Test_Arm extends LinearOpMode{
    //變數設定

    private Servo ArmLeft, ArmRight;
    private Servo OutputClaw;
    private double ClawMaximum = 0.85;
    private double ClaxMinimum = 0.68;

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {
            // * 迴圈執行內容;
            // *
            if(gamepad2.a) {
                ArmLeft.setPosition(ArmLeft.getPosition() + 0.001);
                ArmRight.setPosition(ArmRight.getPosition() + 0.001);
            }
            if(gamepad2.b) {
                ArmLeft.setPosition(ArmLeft.getPosition() - 0.001);
                ArmRight.setPosition(ArmRight.getPosition() - 0.001);
            }

            if(gamepad2.dpad_up) {
                OutputClaw.setPosition(ClawMaximum);
            }
            if(gamepad2.dpad_down) {
                OutputClaw.setPosition(ClaxMinimum);
            }

            telemetry.addLine("Gamepad2 A / B : Arm Up / Down");
            telemetry.addLine("Gamepad2 X / Y : Claw Open / Close");
            telemetry.addData("OutputClawPosition", OutputClaw.getPosition());
            telemetry.addData("ArmLeftPosition", ArmLeft.getPosition());
            telemetry.addData("ArmRightPosition", ArmRight.getPosition());
            telemetry.update();

            idle();
        }
    }
    // 建立函式
    private void init_hardware() {
        //設定物件
        //初始狀態設定，例如Servo初始位置
        ArmLeft = hardwareMap.get(Servo.class, "ArmLeft");
        ArmRight = hardwareMap.get(Servo.class, "ArmRight");
        ArmLeft.setDirection(Servo.Direction.REVERSE);
        ArmLeft.setPosition(0.2);
        ArmRight.setPosition(0.2);

        OutputClaw = hardwareMap.get(Servo.class, "OutputClaw");
        OutputClaw.setPosition(ClawMaximum);
        idle();
    }
}
// 外面不可以寫程式喔!!!