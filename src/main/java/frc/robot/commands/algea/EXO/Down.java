package frc.robot.commands.algea.EXO;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NickClimbingSubsystem;
import frc.robot.subsystems.OzzyGrabberSubsystem;

public class Down extends Command{
    private OzzyGrabberSubsystem climbing;

    public Down(OzzyGrabberSubsystem subsystem) {
        // Initialization code here
        climbing = subsystem;
        addRequirements(climbing);
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
        if(climbing.down()) {
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Code to stop the elevator
    }
}
