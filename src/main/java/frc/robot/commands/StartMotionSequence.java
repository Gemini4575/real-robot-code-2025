package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.datamodel.MotionDirective;
import frc.robot.service.MotionService;

public class StartMotionSequence extends Command {
    private final MotionService motionService;
    private final MotionDirective[] motions;
    private boolean finished = false;

    public StartMotionSequence(MotionService motionService, MotionDirective ... motions) {
        this.motionService = motionService;
        this.motions = motions;
    }

    @Override
    public void initialize() {
        finished = false;
        motionService.executeSequence(motions);
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
