package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonomousMode")
public class AutonomousMode extends Autonomous_Base {
    //變數設定
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        //anything you want can be written here
        DriveStraight(500, 0, 60, 2);
    }
}