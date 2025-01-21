package pedroPathing.CommandPackage.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import pedroPathing.CommandPackage.RobotConstants;
import pedroPathing.CommandPackage.Subsystems.OutputSubsystem;

public class TransferArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final OutputSubsystem outputSubsystem_command;

    public TransferArmCommand(OutputSubsystem outputSubsystem) {
        outputSubsystem_command = outputSubsystem;
        addRequirements(outputSubsystem_command);
    }

    @Override
    public void initialize() {
        outputSubsystem_command.setOutputPosition(
                RobotConstants.TransferArmCommand_ArmPosition,
                RobotConstants.TransferArmCommand_CenterPosition,
                RobotConstants.TransferArmCommand_ClawPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
