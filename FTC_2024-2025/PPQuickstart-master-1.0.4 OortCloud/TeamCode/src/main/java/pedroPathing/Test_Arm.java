package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test_Arm", group = "Test")
public class Test_Arm extends LinearOpMode{
    //變數設定

    CRServo ArmLeft, ArmRight;
    Servo claw;

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {
            // * 迴圈執行內容


            telemetry.addData("servoRight", .getPosition());
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
        ArmLeft = hardwareMap.get(CRServo.class, "ArmLeft");
        ArmRight = hardwareMap.get(CRServo.class, "ArmRight");
        ArmRight.setDirection(CRServo.Direction.REVERSE);
        ArmRight.setMode(CRServo.class, "RUN_WITHOUT_ENCODER");


        claw = hardwareMap.get(Servo.class, "claw");


        idle();
    }
}
// 外面不可以寫程式喔!!!