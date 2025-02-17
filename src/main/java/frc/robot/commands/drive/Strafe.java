package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.service.DriveService;
import frc.robot.subsystems.drive.DriveTrain;

public class Strafe extends Command {

    private final DriveService driveService;
    private final double meters;

    public Strafe(DriveTrain driveTrain, double meters) {
        driveService = new DriveService(driveTrain);
        this.meters = meters;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveService.startDriving(5, meters, 0.005);
    }

    @Override
    public void execute() {
        driveService.keepDriving();
    }

    @Override
    public boolean isFinished() {
        return !driveService.keepDriving();
    }

}
