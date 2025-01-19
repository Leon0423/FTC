package pedroPathing.CommandPackage.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import pedroPathing.CommandPackage.RobotConstants;
import pedroPathing.CommandPackage.Subsystems.OutputSubsystem;

public class BasketArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final OutputSubsystem outputSubsystem_command;

    public BasketArmCommand(OutputSubsystem outputSubsystem) {
        outputSubsystem_command = outputSubsystem;
        addRequirements(outputSubsystem_command);
    }

    @Override
    public void initialize() {
        outputSubsystem_command.setOutputPosition(
                RobotConstants.BasketArmCommand_ArmPosition,
                RobotConstants.BasketArmCommand_CenterPosition,
                RobotConstants.BasketArmCommand_ClawPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}