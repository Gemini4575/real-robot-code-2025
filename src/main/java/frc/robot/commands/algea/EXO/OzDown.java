package frc.robot.commands.algea.EXO;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OzzyGrabberSubsystem;

public class OzDown extends Command{
    OzzyGrabberSubsystem grabber;
    public OzDown(OzzyGrabberSubsystem subsystem) {
        grabber = subsystem;
        addRequirements(grabber);
    }
    boolean isFinished;
    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
    @Override
    public void execute() {
        if(grabber.down()) {
            isFinished = true;
        }
    }
}
