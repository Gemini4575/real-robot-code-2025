package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.service.DriveService;
import frc.robot.subsystems.drive.DriveTrain;

public class DriveXMeters extends Command {
    
    double meters = 0.0;

    int drivemethod;
    private boolean isFinished = false;
    private final DriveService driveService;

    public DriveXMeters(DriveTrain driveTrain, double meters, int DRIVEMETHOD) {
        driveService = new DriveService(driveTrain);
        this.meters = meters;
        this.drivemethod = DRIVEMETHOD;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        isFinished = false;
        driveService.startDriving(drivemethod, meters, 0.005);
    }

    @Override
    public void execute() {
        if(!driveService.keepDriving()) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
