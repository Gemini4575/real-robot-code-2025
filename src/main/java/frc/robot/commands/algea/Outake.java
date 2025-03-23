package frc.robot.commands.algea;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OzzyGrabberConstants;
import frc.robot.subsystems.OzzyGrabberSubsystem;

public class Outake extends Command{
    OzzyGrabberSubsystem o;
    public Outake(OzzyGrabberSubsystem z) {
        o = z;
        addRequirements(z);
    }

    
}