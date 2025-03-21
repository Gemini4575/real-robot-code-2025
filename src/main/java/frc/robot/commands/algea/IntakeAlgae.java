// package frc.robot.commands.algea;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.algea.EXO.IntakeFloorAlgae;
// import frc.robot.commands.algea.EXO.IntakeLolipopAlgae;
// import frc.robot.subsystems.OzzyGrabberSubsystem;

// public class IntakeAlgae extends Command{
//     OzzyGrabberSubsystem grabber;
//     public IntakeAlgae(OzzyGrabberSubsystem subsystem){
//         this.grabber = subsystem;
//         addRequirements(grabber);
//     }
//     @Override
//     public void execute() {
//         SequentialCommandGroup commandGroup = new SequentialCommandGroup(
//             new IntakeLolipopAlgae(grabber).withTimeout(3),
//             new IntakeFloorAlgae(grabber).onlyIf(grabber.BeamBreak())
//         );
//         commandGroup.schedule();
//         if(commandGroup.isFinished()) {
//             this.end(false);
//         }
//     }
// }
