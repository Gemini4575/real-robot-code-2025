package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral.lili.LIPlaceCoral;
import frc.robot.commands.drive.DriveXMeters;
import frc.robot.subsystems.LiliCoralSubystem;
import frc.robot.subsystems.drive.DriveTrain;

public class DriveAndDropToOne extends SequentialCommandGroup{
    public DriveAndDropToOne(DriveTrain s, LiliCoralSubystem c) {
        addCommands(
            new DriveXMeters(s, Units.inchesToMeters(75), 3),
            new LIPlaceCoral(c)
        );
    }
}
