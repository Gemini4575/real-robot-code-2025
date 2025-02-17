package frc.robot.commands.algea.EXO;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OzzyGrabberSubsystem;

public class OzIntake extends Command{
    OzzyGrabberSubsystem grabber;
    public OzIntake(OzzyGrabberSubsystem subsystem) {
        this.grabber = subsystem;
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
        if(grabber.intake()){
            isFinished = true;
        }
    }
}
