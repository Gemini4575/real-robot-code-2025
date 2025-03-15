package frc.robot.commands.drive;

import static frc.robot.Constants.Vision.kTagLayout;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveTrain;

public class PatrolCoralStations extends Command {

    private boolean originalDirection = true;
    private Pose2d startPose;
    private Pose2d endPose;
    private Command cmd;

    private final DriveTrain driveSubsystem;

    public PatrolCoralStations(DriveTrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        // identify the start and end poses for the patrol
        switch(DriverStation.getAlliance().orElse(Alliance.Blue)) {
            case Red:
                startPose = kTagLayout.getTagPose(12).orElse(new Pose3d()).toPose2d();
                endPose = kTagLayout.getTagPose(13).orElse(new Pose3d()).toPose2d();
                break;
            case Blue:
            default:
                startPose = kTagLayout.getTagPose(1).orElse(new Pose3d()).toPose2d();
                endPose = kTagLayout.getTagPose(2).orElse(new Pose3d()).toPose2d();
                break;
        }
        // determine if we need to change direction
        originalDirection ^= needToTurnAround();
        startMoving();
    }

    private void startMoving() {
        var constraints = new PathConstraints(4, 3, 300, 720);
        cmd = AutoBuilder.pathfindToPose(originalDirection ? endPose : startPose, constraints);
        cmd.schedule();
    }
    
    @Override
    public void execute() {
        if((cmd != null && cmd.isFinished()) || needToTurnAround()) {            
            cmd.cancel();
            originalDirection = !originalDirection; // toggle direction
            startMoving();
        }
    }

    @Override
    public boolean isFinished() {
        return cmd == null;
    }

    @Override
    public void end(boolean interrupted) {
        if(cmd != null) {
            cmd.cancel();
            cmd = null;
        }
    }
    
    private boolean needToTurnAround() {
        var targetPose = originalDirection ? endPose : startPose;
        return driveSubsystem.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.8;
    }

}
