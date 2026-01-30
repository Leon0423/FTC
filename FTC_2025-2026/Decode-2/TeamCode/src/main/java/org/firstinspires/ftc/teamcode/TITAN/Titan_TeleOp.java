package org.firstinspires.ftc.teamcode.TITAN;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Titan", group = "Titan")
public class Titan_TeleOp extends Titan_Base {

    boolean lastB, lastX;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();

        while (opModeIsActive()) {
            // 持續執行移動控制
            moveRobot();
            intake();

            // 更新里程計
            odo.update();

            // 更新射擊狀態機（非阻塞）
            updateShooting();

            // 按下 X 或 B 時啟動射擊
            if (gamepad1.x && !lastX) {
                startShootingf();  // 遠距射擊
            }
            if (gamepad1.b && !lastB) {
                startShootingc();  // 近距射擊
            }

            lastX = gamepad1.x;
            lastB = gamepad1.b;

            // 顯示資訊
            Pose2D pos = odo.getPosition();
            telemetry.addData("Robot X", odo.getPosX(MM));
            telemetry.addData("Robot Y", odo.getPosY(MM));
            telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
