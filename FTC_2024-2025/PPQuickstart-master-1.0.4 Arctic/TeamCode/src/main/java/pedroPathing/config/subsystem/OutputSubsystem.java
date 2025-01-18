package pedroPathing.config.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OutputSubsystem extends SubsystemBase {
    // 1. 宣告硬體組件
    private Servo OutputClaw, OutputCenter, OutputLeft, OutputRight;

    // 2. 定義常數
    private static final double DEFAULT_POWER = 0.5;

    // 3. 建構子 - 初始化硬體
    public OutputSubsystem(HardwareMap hardwareMap) {
        // 取得馬達參考
        OutputClaw = hardwareMap.get(Servo.class, "OutputClaw");
        OutputCenter = hardwareMap.get(Servo.class, "OutputCenter");
        OutputLeft = hardwareMap.get(Servo.class, "OutputLeft");
        OutputLeft.setDirection(Servo.Direction.FORWARD);
        OutputRight = hardwareMap.get(Servo.class, "OutputRight");
        OutputRight.setDirection(Servo.Direction.REVERSE);

        // 設定馬達基本配置
        OutputClaw.setPosition(0);
        OutputCenter.setPosition(0);
        OutputLeft.setPosition(0);
        OutputRight.setPosition(0);
    }

    // 4. periodic 方法 - 每個控制迴圈都會執行
    @Override
    public void periodic() {
        // 在這裡放入需要持續檢查的邏輯
        // 例如: 安全檢查、狀態更新等
    }

    // 5. 控制方法
    public void outputClaw(double Position) {
        OutputClaw.setPosition(Position);
    }

    public void outputCenter(double Position) {
        OutputCenter.setPosition(Position);
    }

    public void outputLeft(double Position) {
        OutputLeft.setPosition(Position);
    }

    public void outputRight(double Position) {
        OutputRight.setPosition(Position);
    }

    public void setDefaultPosition() {
        outputClaw(0.15);
        outputCenter(0);
        outputLeft(0);
        outputRight(0);
    }

    public void BasketPlacement() {
        while(OutputRight.getPosition() < 0.8 && OutputLeft.getPosition() < 0.8) { // 0.8 is the Target position
            outputLeft(0.8);
            outputRight(0.8);
            outputCenter(0.2);
        }
        OpenClaw();
    }

    public void SpecimenGrabPosition(double clawPosition, double centerPosition, double leftPosition, double rightPosition) {
        outputClaw(clawPosition);
        outputCenter(centerPosition);
        outputLeft(leftPosition);
        outputRight(rightPosition);
    }

    public void SpecimenPlacement() {
        double SpecimenTargetPlacement = 0.3;
        while(OutputRight.getPosition() < SpecimenTargetPlacement  && OutputLeft.getPosition() < SpecimenTargetPlacement) { // 0.8 is the Target position
            outputLeft(0.8);
            outputRight(0.8);
            outputCenter(0.2);
        }
        OpenClaw();
    }

    public void OpenClaw() {
        outputClaw(0.13);
    }


    // 6. 取得狀態的方法
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
