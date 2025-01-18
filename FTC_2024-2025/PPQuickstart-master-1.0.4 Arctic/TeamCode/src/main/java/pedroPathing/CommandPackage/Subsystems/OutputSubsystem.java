package pedroPathing.CommandPackage.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.CommandPackage.RobotConstants;

public class OutputSubsystem extends SubsystemBase {
    // * 宣告硬體組件
    private final Servo OutputClaw, OutputCenter, OutputLeft, OutputRight;

    // * 建構子 - 初始化硬體
    public OutputSubsystem(final HardwareMap hardwareMap) {
        // 取得馬達參考
        OutputClaw = hardwareMap.get(Servo.class, "OutputClaw");
        OutputCenter = hardwareMap.get(Servo.class, "OutputCenter");
        OutputLeft = hardwareMap.get(Servo.class, "OutputLeft");
        OutputLeft.setDirection(Servo.Direction.FORWARD);
        OutputRight = hardwareMap.get(Servo.class, "OutputRight");
        OutputRight.setDirection(Servo.Direction.REVERSE);

        // * 設定馬達基本配置
        setDefaultPosition();
    }

    // * 初始化方法 - 設定硬體的初始狀態
    public void setDefaultPosition() {
        OpenClaw();
        setOutputCenter(0);
        setOutputLeft(0);
        setOutputRight(0);
    }

    // * 控制方法
    public void setArmPosition(double RotatePosition) {
        setOutputLeft(RotatePosition);
        setOutputRight(RotatePosition);
    }

    public void OpenClaw() {
        setOutputClaw(RobotConstants.OpenOutputClawPosition);
    }

    // * 控制方法
    public void setOutputClaw(double Position) {
        OutputClaw.setPosition(Position);
    }

    public void setOutputCenter(double Position) {
        OutputCenter.setPosition(Position);
    }

    public void setOutputLeft(double Position) {
        OutputLeft.setPosition(Position);
    }

    public void setOutputRight(double Position) {
        OutputRight.setPosition(Position);
    }

    // * 取得狀態的方法
    public double getOutputClawCurrentPosition() {
        return OutputClaw.getPosition();
    }

    public double getOutputCenterCurrentPosition() {
        return OutputCenter.getPosition();
    }

    public double getOutputLeftCurrentPosition() {
        return OutputLeft.getPosition();
    }

    public double getOutputRightCurrentPosition() {
        return OutputRight.getPosition();
    }
}
