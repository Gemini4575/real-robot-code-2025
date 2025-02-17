package frc.robot.commands.algea.EXO;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OzzyGrabberSubsystem;

public class OzUp extends Command{
    OzzyGrabberSubsystem grabber;
    public OzUp(OzzyGrabberSubsystem subsystem) {
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
        // Code to move the elevator
        if (grabber.up()) {
            isFinished = true;
        }
    }
    @Override
    public void end(boolean interrupted) {
        // Code to stop the elevator
        
    }
}
