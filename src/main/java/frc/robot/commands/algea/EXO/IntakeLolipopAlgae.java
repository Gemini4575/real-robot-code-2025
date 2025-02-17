package frc.robot.commands.algea.EXO;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.OzzyGrabberSubsystem;

public class IntakeLolipopAlgae extends SequentialCommandGroup{
    public IntakeLolipopAlgae(OzzyGrabberSubsystem grabber) {
        addCommands(
            new OzDown(grabber),
            new OzIntake(grabber),
            new OzUp(grabber)
        );
    }
}
