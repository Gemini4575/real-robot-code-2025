package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NickClimbingSubsystem;

public class Climb extends Command{
    
    private NickClimbingSubsystem climbing;

    public Climb(NickClimbingSubsystem subsystem) {
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
        if(climbing.Climb()) {
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Code to stop the elevator
    }
    
}
