// package frc.robot.commands.drive;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.drive.DriveTrain;

// public class TestTurnCommand extends Command {

//     private final DriveTrain driveTrain;
//     private final double turnAngle;
//     private double timeStarted;

//     public TestTurnCommand(DriveTrain driveTrain, double turnAngle) {
//         this.driveTrain = driveTrain;
//         this.turnAngle = turnAngle;
//     }

//     @Override
//     public void initialize() {
//         driveTrain.drive(0.5, 0, 0, false, true);
//         timeStarted = System.currentTimeMillis();
//     }

//     @Override
//     public void execute() {
//         driveTrain.drive(0.5, 0, 0, false, true);
//     }

//     @Override
//     public boolean isFinished() {
//         return System.currentTimeMillis() > timeStarted + 5000;
//     }

// }