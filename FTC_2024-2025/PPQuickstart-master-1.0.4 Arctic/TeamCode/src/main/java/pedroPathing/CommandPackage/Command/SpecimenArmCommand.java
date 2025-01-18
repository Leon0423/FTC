package pedroPathing.CommandPackage.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.R;

import pedroPathing.CommandPackage.RobotConstants;
import pedroPathing.CommandPackage.Subsystems.OutputSubsystem;

public class SpecimenArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final OutputSubsystem outputSubsystem_command;
    private final double SpecimenArmPosition;
    private final double SpecimenCenterPosition;

    public SpecimenArmCommand(OutputSubsystem outputSubsystem) {
        outputSubsystem_command = outputSubsystem;
        this.SpecimenArmPosition = RobotConstants.SpecimenArmPosition;
        this.SpecimenCenterPosition = RobotConstants.SpecimenCenterPosition;
        addRequirements(outputSubsystem_command);
    }

    @Override
    public void initialize() {
        outputSubsystem_command.setArmPosition(SpecimenArmPosition);
        outputSubsystem_command.setOutputCenter(SpecimenCenterPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
