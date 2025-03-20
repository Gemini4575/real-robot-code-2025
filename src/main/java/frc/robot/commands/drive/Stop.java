package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveTrain;

public class Stop extends Command{
    DriveTrain d;
    boolean isFinished;
    Timer timer;
    public Stop(DriveTrain d) {
        this.d = d;
        addRequirements(d);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        isFinished = false;
        timer.start();
    }

    @Override
    public void execute() {
        d.stop();
        if(timer.advanceIfElapsed(.5)) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}