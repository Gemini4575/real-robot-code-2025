package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveTrain;

import java.util.function.Supplier;

public class PathFindToPose extends Command {

    private final DriveTrain driveSubsystem; 
    private final Supplier<Pose2d> targetPoseSuppler;

    private Command cmd;

    public PathFindToPose(DriveTrain driveSubsystem, Supplier<Pose2d> targetPoseSuppler) {
        this.driveSubsystem = driveSubsystem;
        this.targetPoseSuppler = targetPoseSuppler;      
    }

    @Override
    public void initialize() {
        System.out.println("Initializing PathFindToPose command");
        cmd = AutoBuilder.pathfindToPose(
                targetPoseSuppler.get(),
                driveSubsystem.getChassisConstrains());
        cmd.schedule();
        System.out.println("Initializing PathFindToPose command - DONE");
    }

    @Override
    public boolean isFinished() {
        return cmd == null || cmd.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended PathFindToPose command");
        cmd = null;
    }
    
}
