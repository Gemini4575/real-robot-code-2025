package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveTrain;


public class TelopSwerve extends Command {    
    private DriveTrain s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier slowMode;
    private boolean isFinished;
    private double slowModeFactor = 1.0;
    double i = 0.0;
    /**
     * This is the command that dirves your swerve robot. When setting this up in robot container
     * use the {@link DriveTrain#setDefaultCommand}.
     * When putting in your paramenters use lamda code i.e. () -> {@link DoubleSupplier}, 
     * instead of just {@link DoubleSupplier}.
     * 
     * @param s_Swerve you drivetrain subsystem
     * @param translationSup the axies that controlls the foward/backwards movment of you robot
     * @param strafeSup the axies that controlls the side to side movment of you robot
     * @param rotationSup the axies that controlls the angle of your robot
     */
    public TelopSwerve(DriveTrain s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, 
    DoubleSupplier rotationSup, BooleanSupplier slowMode) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        isFinished = false;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.slowMode = slowMode;
    }

    @Override
    public void execute() {
        isFinished = false;
        if(slowMode.getAsBoolean()) {
            if(DriverStation.getMatchTime() >= 120.0) {
                slowModeFactor = 0.25;
            } else {
                slowModeFactor = 0.5;
            }
        } else {
            slowModeFactor = 1.0;
        }

        /* Get Values, Deadband*/
        double strafeVal = MathUtil.applyDeadband(translationSup.getAsDouble(), 0.2) * -slowModeFactor;
        double translationVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.3) * slowModeFactor;
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.4) * slowModeFactor;

        /* Drive */
        if (!RobotState.isAutonomous()) {
            s_Swerve.drive(
            strafeVal,
            translationVal,
            rotationVal,
            true
            );
        }
    }

    @Override
    public void end(boolean interupted) {}

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}