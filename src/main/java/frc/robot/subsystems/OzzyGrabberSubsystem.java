package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OzzyGrabberConstants;

public class OzzyGrabberSubsystem extends SubsystemBase{
    SparkMax GrabberMotor;
    SparkMax PoseMotor;
    DigitalInput AlgeaSensor;
    private long algaeLastSeenTime;
    private AnalogInput analogSensor;

    public OzzyGrabberSubsystem() {
        // Initialization code here
        GrabberMotor = new SparkMax(OzzyGrabberConstants.GrabberMotor, MotorType.kBrushless);
        //GrabberMotor = new Motor(OzzyGrabberConstants.GrabberMotor, MotorType.kBrushless);
        PoseMotor = new SparkMax(OzzyGrabberConstants.PosetionMotor, MotorType.kBrushless);
        AlgeaSensor = new DigitalInput(OzzyGrabberConstants.BeamBreak);
        analogSensor = new AnalogInput(4); //this works but we have no more analog ports
    }

    public boolean Grab() {
        // Code to move the elevator
        if(PoseMotor.getEncoder().getPosition() < OzzyGrabberConstants.MiddleLength) {
            PoseMotor.set(OzzyGrabberConstants.DownSpeed);
        } else {
            PoseMotor.set(0);
        }
        return PoseMotor.getEncoder().getPosition() == OzzyGrabberConstants.MiddleLength;
    }

    public boolean intake() {
        if(!AlgeaSensor.get()) {
            GrabberMotor.set(OzzyGrabberConstants.IntakeSpeed);
        } else {
            GrabberMotor.stopMotor();
        }
        return AlgeaSensor.get();
    }

    public boolean outake() {
        if(AlgeaSensor.get()) {
            GrabberMotor.set(OzzyGrabberConstants.OutakeSpeed);
            algaeLastSeenTime = System.currentTimeMillis();
        } else {
            if (System.currentTimeMillis() > algaeLastSeenTime + 500) {
                GrabberMotor.stopMotor();
                return true;
            }
        }
        return false;
    }

    public boolean down() {
        if(PoseMotor.getEncoder().getPosition() < OzzyGrabberConstants.MovmentLength) {
             PoseMotor.set(OzzyGrabberConstants.DownSpeed);
             return false;
        } else {
             PoseMotor.stopMotor();
        }
        return true;
    }

    public boolean up() {
        if(PoseMotor.getEncoder().getPosition() > 0) {
            PoseMotor.set(OzzyGrabberConstants.UpSpeed);
            return false;
        } else {
            PoseMotor.stopMotor();
        }
        return true;
    }

    public boolean isHangingLoose() {
        return PoseMotor.getEncoder().getPosition() > 0.15 * OzzyGrabberConstants.MovmentLength;
    }

    public BooleanSupplier BeamBreak() {
        return () -> AlgeaSensor.get();
    }

    public BooleanSupplier FalseBeamnBreak() {
        return () -> !AlgeaSensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Grabber position", GrabberMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Algae Pose position", PoseMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Algae sensor", AlgeaSensor.get());
        SmartDashboard.putNumber("Algae sensor1", analogSensor.getValue());
        SmartDashboard.putNumber("Algae sensor2", analogSensor.getVoltage());
        SmartDashboard.putNumber("Algae sensor3", analogSensor.getAverageValue());
    }
}
