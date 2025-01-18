package pedroPathing.CommandPackage.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import pedroPathing.CommandPackage.Subsystems.OutputSubsystem;

public class DefaultArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final OutputSubsystem outputSubsystem_command;

    public DefaultArmCommand(OutputSubsystem outputSubsystem) {
        outputSubsystem_command = outputSubsystem;
        addRequirements(outputSubsystem_command);
    }

    @Override
    public void initialize() {
        outputSubsystem_command.setDefaultPosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
