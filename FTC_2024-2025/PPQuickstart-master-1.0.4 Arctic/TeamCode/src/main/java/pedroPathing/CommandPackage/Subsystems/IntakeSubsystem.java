package pedroPathing.CommandPackage.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.CommandPackage.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
    // * 宣告硬體組件
    private final Servo HSArmLeft, HSArmRight, IntakeClaw;

    // * 建構子 - 初始化硬體
    public IntakeSubsystem(final HardwareMap hardwareMap) {
        // 取得馬達參考
        HSArmLeft = hardwareMap.get(Servo.class, "HSArmLeft");
        HSArmRight = hardwareMap.get(Servo.class, "HSArmRight");
        HSArmRight.setDirection(Servo.Direction.REVERSE);

        IntakeClaw = hardwareMap.get(Servo.class, "IntakeClaw");
    }

    // * 設定整體位置
    public void setIntakePosition(double leftPosition, double rightPosition, boolean OpenClaw) {
        HSArmLeft.setPosition(leftPosition);
        HSArmRight.setPosition(rightPosition);
        if (OpenClaw) {
            OpenIntakeClaw();
        } else {
            CloseIntakeClaw();
        }
    }

    // * IntakeClaw_Position
    public void OpenIntakeClaw() {
        IntakeClaw.setPosition(RobotConstants.IntakeSubsystem_OpenClawPosition);
    }

    public void CloseIntakeClaw() {
        IntakeClaw.setPosition(RobotConstants.IntakeSubsystem_CloseClawPosition);
    }

    // * setArm_Position
    public void setHSArmPosition(double position) {
        HSArmLeft.setPosition(position);
        HSArmRight.setPosition(position);
    }

    // * setClaw_Position
    public void setIntakeClawPosition(double position) {
        IntakeClaw.setPosition(position);
    }

    // * Single: setRightArm_Position
    public void setIntakeRightPosition(double position) {
        HSArmRight.setPosition(position);
    }

    // * Single: setLeftArm_Position
    public void setIntakeLeftPosition(double position) {
        HSArmLeft.setPosition(position);
    }

    // * getArm_Position
    public double getHSArmPosition() {
        return HSArmLeft.getPosition();
    }

    // * getRightArm_Position
    public double getIntakeRightPosition() {
        return HSArmRight.getPosition();
    }

    // * getLeftArm_Position
    public double getIntakeLeftPosition() {
        return HSArmLeft.getPosition();
    }

    // * getClaw_Position
    public double getIntakeClawPosition() {
        return IntakeClaw.getPosition();
    }

}
