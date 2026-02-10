package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "ServoTest", group="Iterative")
public class ServoTest extends LinearOpMode {

    CRServo servo;
    AnalogInput encoder;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(CRServo.class, "servo");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 讀取編碼器電壓 (通常範圍是 0-3.3V)
            double voltage = encoder.getVoltage();

            // 將電壓轉換為角度 (0-3.3V 對應 0-360度)
            // 假設編碼器是絕對式編碼器，電壓線性對應角度
            double angle = (voltage / 3.3) * 360.0;

            // 也可以計算弧度
            double radians = Math.toRadians(angle);

            telemetry.addData("Status", "Running");
            telemetry.addData("Encoder Voltage", "%.3f V", voltage);
            telemetry.addData("Angle (Degrees)", "%.2f°", angle);
            telemetry.addData("Angle (Radians)", "%.3f rad", radians);
            telemetry.update();

        }
    }
}
