// package frc.robot.commands.algea;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.commands.algea.EXO.OzDown;
// import frc.robot.commands.algea.EXO.OzOutake;
// import frc.robot.commands.algea.EXO.OzUp;
// import frc.robot.subsystems.OzzyGrabberSubsystem;

// public class Proceser extends SequentialCommandGroup{
//     public Proceser(OzzyGrabberSubsystem grabber, BooleanSupplier wait) {
//         addCommands(
//             new OzDown(grabber),
//             new WaitUntilCommand(wait),
//             new OzOutake(grabber),
//             new WaitUntilCommand(wait),
//             new OzUp(grabber)
//         );
//     }
// }
