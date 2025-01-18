package pedroPathing.CommandPackage.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import pedroPathing.CommandPackage.RobotConstants;
import pedroPathing.CommandPackage.Subsystems.OutputSubsystem;

public class BasketArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final OutputSubsystem outputSubsystem_command;
    private final double BasketArmPosition;
    private final double BasketCenterPosition;

    public BasketArmCommand(OutputSubsystem outputSubsystem) {
        outputSubsystem_command = outputSubsystem;
        this.BasketArmPosition = RobotConstants.BasketArmPosition;
        this.BasketCenterPosition = RobotConstants.BasketCenterPosition;
        addRequirements(outputSubsystem_command);
    }

    @Override
    public void initialize() {
        outputSubsystem_command.setArmPosition(BasketArmPosition);
        outputSubsystem_command.setOutputCenter(BasketCenterPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}