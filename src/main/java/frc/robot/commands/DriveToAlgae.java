package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.GeometryHelper;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.DriveTrain;

public class DriveToAlgae extends Command {

    private static final double MIN_ALGAE_DISTANCE = 1.0; // closest we will try to drive in meters

    private final double VISION_TURN_kP = 0.01;
    private final double VISION_DES_ANGLE_deg = 0.0;
    private final double VISION_STRAFE_kP = 0.5;

    private final Vision vision;
    private final DriveTrain driveTrain;

    private boolean isTargetVisible = true;
    private double targetDistance = Double.MAX_VALUE;

    public DriveToAlgae(DriveTrain driveTrain, Vision vision) {
        this.driveTrain = driveTrain;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("DriveToAlgae running", true);
        System.out.println("Initializing DriveToAlgae command");
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("DriveToAlgae running", false);
        System.out.println("Ended DriveToAlgae command");
    }

    @Override
    public void execute() {
        var target = vision.getAlgaeTarget();
        if (target != null) {
            var targetYaw = target.getYaw();
            targetDistance = GeometryHelper.distanceToAlgae(target.getArea());

            var turn = (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * DriveTrain.kMaxAngularSpeed;
            var forward = -targetDistance* VISION_STRAFE_kP * DriveTrain.kMaxSpeed;

            SmartDashboard.putBoolean("Algae Target is visible", true);
            SmartDashboard.putNumber("Algae Target Yaw", targetYaw);
            SmartDashboard.putNumber("Algae Target Distance", targetDistance);
            SmartDashboard.putNumber("Algae Target drive turn", turn);
            SmartDashboard.putNumber("Algae Target drive forward", forward);

            System.out.println("DriveToAlgae command - distance to Algae = " + targetDistance);
            System.out.println("DriveToAlgae command - yaw to Algae = " + targetYaw);
            System.out.println("DriveToAlgae command - turn to Algae = " + turn);
            System.out.println("DriveToAlgae command - drive to Algae = " + forward);

            driveTrain.drive(forward, 0, turn, false);
        } else {
            SmartDashboard.putBoolean("Algae Target is visible", false);
            isTargetVisible = false;
        }
    }

    @Override
    public boolean isFinished() {
        return !isTargetVisible || targetDistance < MIN_ALGAE_DISTANCE;
    }
    
}
