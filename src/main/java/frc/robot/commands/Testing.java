package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestingSubsystem;

public class Testing extends Command{
    TestingSubsystem c;
    public Testing(TestingSubsystem d) {
        c = d;
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
        if(c.resetEncoder()) {
            isFinished = true;
        }
    }
}
