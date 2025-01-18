package pedroPathing.CommandPackage.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import pedroPathing.CommandPackage.RobotConstants;
import pedroPathing.CommandPackage.Subsystems.SlideSubsystem;

public class DefaultSlideCommand extends CommandBase {
    private final SlideSubsystem SlideSubsystem_command;

    public DefaultSlideCommand(SlideSubsystem slideSubsystem) {
        SlideSubsystem_command = slideSubsystem;
        addRequirements(SlideSubsystem_command);
    }

    @Override
    public void initialize() {
        SlideSubsystem_command.setSlidePower(RobotConstants.SlidePower);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
