package frc.robot.commands.coral.lili;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LiliCoralSubystem;

public class LIPlaceCoral extends SequentialCommandGroup{
    public LIPlaceCoral(LiliCoralSubystem c) {
        addCommands(
            new EXODropGate(c),
            new WaitCommand(.5),
            new EXOCloseGateSlow(c).withTimeout(5.5)
        );
    }
}
