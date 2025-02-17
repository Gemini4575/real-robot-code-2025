package frc.robot.commands.coral.nora;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoraArmSubsystem;

public class CoralStation extends Command {

    private final NoraArmSubsystem elevator;

    public CoralStation(NoraArmSubsystem subsystem) {
        elevator = subsystem;
        addRequirements(elevator);
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
        if (elevator.CoralStation()) {
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Code to stop the elevator
        elevator.stop();
    }

}
