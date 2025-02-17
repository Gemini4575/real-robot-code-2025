package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NickClimbingSubsystem;

public class init extends Command{
    
    private NickClimbingSubsystem climbing;

    public init(NickClimbingSubsystem subsystem) {
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
        if(climbing.init1()&&climbing.init2()) {
            isFinished = true;
        }
    }
}
