package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveTrain;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.json.simple.parser.ParseException;

public class PathFindToPose extends Command {

    public static enum PathTarget {
        ALGAE_INTAKE,
        LEFT_HUMAN_STATION
    }

    private static final Map<PathTarget, String> PATH_BY_TARGET = new HashMap<>();
    static {
        PATH_BY_TARGET.put(PathTarget.ALGAE_INTAKE, "To_Algae_Intake");
        PATH_BY_TARGET.put(PathTarget.LEFT_HUMAN_STATION, "To_Left_Human");
    }

    private final DriveTrain driveSubsystem; 
    private PathPlannerPath path;
    private boolean finished = true;

    public PathFindToPose(DriveTrain driveSubsystem, PathTarget pathTarget) {
        this.driveSubsystem = driveSubsystem;
        try {
            this.path = PathPlannerPath.fromPathFile(PATH_BY_TARGET.get(pathTarget));
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.println("PathFindToPose Could not load paths");
            e.printStackTrace();
        }     
    }

    @Override
    public void initialize() {
        finished = false;
        System.out.println("Initializing PathFindToPose command");
        Command cmd = AutoBuilder.pathfindThenFollowPath(path, driveSubsystem.getChassisConstrains());
        cmd.schedule();
        System.out.println("Initializing PathFindToPose command - DONE");
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended PathFindToPose command");
    }
    
}
