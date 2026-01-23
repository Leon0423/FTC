package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends TeleOpMode_Base{
    //變數設定
    double drive, turn, strafe;
    double fr, fl, br, bl, scale;

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        //初始狀態設定，例如Servo初始位置
        while(opModeIsActive()) {
            //迴圈執行內容
            drive = -gamepad1.left_stick_y;     // * 前進
            turn = gamepad1.right_stick_x;      // * 自旋
            strafe = gamepad1.left_stick_x;    // * 平移

            scale = scaling_power(fr, fl, br, bl); // * 取得最大值

            FR.setPower((drive - turn - strafe)/scale);
            FL.setPower((drive + turn + strafe)/scale);
            BR.setPower((drive - turn + strafe)/scale);
            BL.setPower((drive + turn - strafe)/scale);

            idle();
        }
    }
    // 建立函式
    public double scaling_power(double fr, double fl, double br, double bl) {
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if(max <= 1) {
            max = 1;
        }
        return max;
    }
}
// 外面不可以寫程式喔!!!