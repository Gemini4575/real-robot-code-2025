package frc.robot.commands.drive;


import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.DriveTrain;

public class DriveTwoardsAprillTag extends Command {
    Vision vision;
    DriveTrain driveTrain;

    public DriveTwoardsAprillTag(Vision vision, DriveTrain driveTrain) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        addRequirements(vision, driveTrain);
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
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

        PhotonTrackedTarget tagTarget = vision.getTagTarget();
        if (tagTarget == null || tagTarget.area > 5) {
            chassisSpeeds.vxMetersPerSecond = 0;
            chassisSpeeds.vyMetersPerSecond = 0;
            isFinished = true;
            driveTrain.stop();
            return;
        } else {
            chassisSpeeds.vyMetersPerSecond = 0.05;
        }
        
        if (Math.abs(tagTarget.getYaw()) > 10) {
            chassisSpeeds.vxMetersPerSecond = ((tagTarget.getYaw() - 1) / 1000);
        } else {
            chassisSpeeds.vxMetersPerSecond = 0;
        }
        chassisSpeeds.omegaRadiansPerSecond = 0;
        driveTrain.driveRobotRelative(chassisSpeeds);
    }
}