package pedroPathing.CommandPackage.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import pedroPathing.CommandPackage.RobotConstants;
import pedroPathing.CommandPackage.Subsystems.OutputSubsystem;

public class SpecimenArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final OutputSubsystem outputSubsystem_command;

    public SpecimenArmCommand(OutputSubsystem outputSubsystem) {
        outputSubsystem_command = outputSubsystem;
        addRequirements(outputSubsystem_command);
    }

    @Override
    public void initialize() {
        outputSubsystem_command.setOutputPosition(
                RobotConstants.SpecimenArmCommand_ArmPosition,
                RobotConstants.SpecimenArmCommand_CenterPosition,
                RobotConstants.SpecimenArmCommand_ClawPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
