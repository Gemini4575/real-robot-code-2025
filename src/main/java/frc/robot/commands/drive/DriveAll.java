package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveTrain;

public class DriveAll extends Command{
    DriveTrain d;
    DoubleSupplier tras;
    DoubleSupplier strafe;
    DoubleSupplier Rot;
    Command one;
    Command two;
    Command three;
    public DriveAll(DriveTrain D, DoubleSupplier tras, DoubleSupplier strafe, DoubleSupplier Rot) {
        this.d = D;
        this.tras = tras;
        this.strafe = strafe;
        this.Rot = Rot;
        addRequirements(d);
    }
    boolean isFinished = false;
    @Override
    public void initialize() {
        isFinished = false;
        one = new DriveStraight(d, tras.getAsDouble());
        one.schedule();
        two = new Turn(d, Rot.getAsDouble());
        two.schedule();
        three = new Strafe(d, strafe.getAsDouble());
        three.schedule();
    }

    @Override
    public void execute() {
        if(one.isFinished() && two.isFinished() && three.isFinished()) {
            isFinished = true;
        } else {
            isFinished = false;
        }
    }

}
