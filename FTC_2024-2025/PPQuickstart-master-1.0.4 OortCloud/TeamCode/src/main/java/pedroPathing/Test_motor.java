package pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test_SlideTargetPosition", group = "Test")
public class Test_motor extends LinearOpMode{
    //變數設定

    DcMotorEx SlideLeft, SlideRight;
    private int SlideMaximumPosition = 3800;
    private int SlideMinimumPosition = 0;
    private int SlidePosition = 0;
    private double SlidePower = 0.3;


    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {
            // * 迴圈執行內容
            // ! gamepad "2" is the second controller
            if(gamepad2.a && SlidePosition < SlideMaximumPosition) {
                SlidePosition += 1;
            } else if (gamepad2.b && SlidePosition > SlideMinimumPosition) {
                SlidePosition -= 1;
            }

            SlideLeft.setTargetPosition(SlidePosition);
            SlideRight.setTargetPosition(SlidePosition);

            telemetry.addData("SlidePosition", SlidePosition);
            telemetry.addData("SlidePower", SlidePower);
            telemetry.addData("Left Encoder Position", SlideLeft.getCurrentPosition());
            telemetry.addData("Right Encoder Position", SlideRight.getCurrentPosition());
            telemetry.addData("Left Target Position", SlideLeft.getTargetPosition());
            telemetry.addData("Right Target Position", SlideRight.getTargetPosition());
            telemetry.addData("SlideMaximumPosition", SlideMaximumPosition);
            telemetry.addData("SlideMinimumPosition", SlideMinimumPosition);
            telemetry.update();
            idle();
        }
    }
    // 建立函式
    private void init_hardware() {
        //設定物件
        //初始狀態設定，例如Servo初始位置
        SlideLeft = hardwareMap.get(DcMotorEx.class, "Left");
        SlideRight = hardwareMap.get(DcMotorEx.class, "Right");
        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        SlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SlideLeft.setTargetPosition(SlidePosition);
        SlideRight.setTargetPosition(SlidePosition);
        SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideLeft.setTargetPosition(SlidePosition);
        SlideRight.setTargetPosition(SlidePosition);
        SlideLeft.setPower(SlidePower);
        SlideRight.setPower(SlidePower);

        idle();
    }
}
// 外面不可以寫程式喔!!!