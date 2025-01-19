package pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test_Slide", group = "Test")
public class Test_motor extends LinearOpMode{
    //變數設定

    DcMotorEx Left, Right;
    private int armUpPosition = 1000;
    private int armDownPosition = 0;
    private int armpos;


    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {
            // * 迴圈執行內容

            Left.setTargetPosition(armpos);
            Right.setTargetPosition(armpos);


            if(gamepad1.a){
                armpos += 1;
                Left.setPower(0.6);
                Right.setPower(0.6);
            } else if (gamepad1.b) {
                armpos -= 1;
                Left.setPower(0.6);
                Right.setPower(0.6);
            }


            // Show the position of the armMotor on telemetry
            telemetry.addData("Left Encoder Position", Left.getCurrentPosition());
            telemetry.addData("Right Encoder Position", Right.getCurrentPosition());


            telemetry.addData("armpos", armpos);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Left Target Position", Left.getTargetPosition());
            telemetry.addData("Right Target Position", Right.getTargetPosition());
            telemetry.update();
            idle();
        }
    }
    // 建立函式
    private void init_hardware() {
        //設定物件
        //初始狀態設定，例如Servo初始位置
        Left = hardwareMap.get(DcMotorEx.class, "Left");
        Right = hardwareMap.get(DcMotorEx.class, "Right");
        Left.setDirection(DcMotorSimple.Direction.REVERSE);
        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Left.setTargetPosition(armDownPosition);
        Right.setTargetPosition(armDownPosition);
        Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armpos = Math.min(Math.max(armpos, armUpPosition), armDownPosition);

        idle();
    }
}
// 外面不可以寫程式喔!!!