package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveTrain;

public class PointWheelsCommand extends Command {

    private final DriveTrain driveTrain;

    public PointWheelsCommand(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        var angleToTurn = SmartDashboard.getNumber("WheelPointAngle", 0);
        SmartDashboard.putNumber("WheelPointRadians", Units.degreesToRadians(angleToTurn));
        var state = new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(angleToTurn)));
        driveTrain.setModuleStates(new SwerveModuleState[]{state, state, state, state});
    }

    
    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }

}
