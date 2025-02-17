package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.service.TurnService;
import frc.robot.subsystems.drive.DriveTrain;

public class Turn extends Command {

    private final TurnService turnService;
    private final double degreesToTurn;

    public Turn(DriveTrain driveTrain, double degreesToTurn) {
        turnService = new TurnService(driveTrain);
        this.degreesToTurn = degreesToTurn;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        turnService.startTurning(degreesToTurn, Math.PI/1800.0);
    }

    @Override
    public void execute() {
        turnService.keepTurning();
    }

    @Override
    public boolean isFinished() {
        return !turnService.keepTurning();
    }

}
