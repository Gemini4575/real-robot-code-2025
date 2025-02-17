package frc.robot.commands.coral.nora;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoraArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class IntakeCoral extends Command{
    
    private NoraArmSubsystem elevator;
    private VisionSubsystem vision;
    

    public IntakeCoral(NoraArmSubsystem subsystem, VisionSubsystem vision) {
        elevator = subsystem;
        this.vision = vision;
        addRequirements(elevator, vision);
    }

    boolean isFinished;
    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }


    @Override
    public void execute() {
        // Code to move the elevator
        if (elevator.intakeCoral(vision.InRange())) {
           isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Code to stop the elevator
        elevator.stop();
    }
}
