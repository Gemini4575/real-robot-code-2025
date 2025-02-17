package frc.robot.commands.algea.EXO;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.OzzyGrabberSubsystem;

public class IntakeFloorAlgae extends SequentialCommandGroup{
    public IntakeFloorAlgae(OzzyGrabberSubsystem grabber){
        addCommands(
            new OzGrab(grabber),
            new OzIntake(grabber),
            new OzUp(grabber)
        );
    }
}
