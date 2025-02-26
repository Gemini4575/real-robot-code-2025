package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.epramotor.Motor;
import frc.robot.Constants;

public class NoraArmSubsystem extends SubsystemBase {

    Motor armMotor;
    Motor TopWheel;
    Motor BottomWheel;
    DigitalInput CoralSensor;

    

    public NoraArmSubsystem() {
        armMotor = new Motor(Constants.ArmConstants.ARM_MOTOR, MotorType.kBrushless);
        TopWheel = new Motor(Constants.ArmConstants.TOP_Wheel);
        BottomWheel = new Motor(Constants.ArmConstants.BOTTOM_Wheel);
        CoralSensor = new DigitalInput(Constants.ArmConstants.CoralSensor);
        BottomWheel.setInverted();
    }

    public void stop() {
        armMotor.set(0);
        TopWheel.set(0);
        BottomWheel.set(0);
    }

    public boolean L1() {
        // Move the arm
        moveArm(armMotor.getEncoder(), Constants.ArmConstants.L1Position);
        return Math.abs(armMotor.getEncoder().getPosition()) == Constants.ArmConstants.L1Position;
    }

    public boolean L2() {
        // Stop the arm
        moveArm(armMotor.getEncoder(), Constants.ArmConstants.L2Position);
        return Math.abs(armMotor.getEncoder().getPosition()) == Constants.ArmConstants.L2Position;
    }

    public boolean L3() {
        // Reset the arm
        moveArm(armMotor.getEncoder(), Constants.ArmConstants.L3Position);
        return Math.abs(armMotor.getEncoder().getPosition()) == Constants.ArmConstants.L3Position;
    }

    public boolean L4() {
        // Move the arm to a specific position
        moveArm(armMotor.getEncoder(), Constants.ArmConstants.L4Position);
        return Math.abs(armMotor.getEncoder().getPosition()) == Constants.ArmConstants.L4Position;
    }

    public boolean CoralStation() {
        // Move the arm to the coral station
        moveArm(armMotor.getEncoder(), Constants.ArmConstants.CoralStationPosition);
        return Math.abs(armMotor.getEncoder().getPosition()) == Constants.ArmConstants.CoralStationPosition;
    }
    
    public boolean intakeCoral(boolean InRange) {
        // Intake the coral
        if (InRange) {
        TopWheel.set(Constants.ArmConstants.IntakeSpeed);
        BottomWheel.set(Constants.ArmConstants.IntakeSpeed);
        } else {
            TopWheel.set(0);
            BottomWheel.set(0);
        }
        if(!CoralSensor.get()){
            TopWheel.set(0);
            BottomWheel.set(0);
            return true;
        }
        return false;
    }

    public void moveArm(RelativeEncoder encoder, double position) {
        if(Math.abs(Math.round(armMotor.getPosition())) <= position) {
            armMotor.set(.5);
        }
    }

}
