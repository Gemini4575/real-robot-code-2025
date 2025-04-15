package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.epramotor.Motor;
import static frc.robot.Constants.NickClimbingConstanst;

public class NickClimbingSubsystem extends SubsystemBase {

    public Motor ClimbingMotor1;
    public Motor ClimbingMotor2;

    public NickClimbingSubsystem() {
        // Initialization code here
        ClimbingMotor1 = new Motor(14, MotorType.kBrushless);
        ClimbingMotor2 = new Motor(13, MotorType.kBrushless);
    }

    private boolean Climb1() {
        // Code to move the elevator
        return ClimbingMotor1.SetIfBoolean(
                Math.abs(ClimbingMotor1.getPosition()) < (NickClimbingConstanst.ClimbingMotorPoseition),
                -NickClimbingConstanst.ClimbingSpeed);
    }

    private boolean Climb2() {
        // Code to move the elevator
        return ClimbingMotor2.SetIfBoolean(
                Math.abs(ClimbingMotor2.getPosition()) < (NickClimbingConstanst.ClimbingMotorPoseition),
                NickClimbingConstanst.ClimbingSpeed);
    }

    public boolean init1() {
        // Code to move the elevator
        if (ClimbingMotor1.getPosition() < Math.round(0)) {
            ClimbingMotor1.set(NickClimbingConstanst.ClimbingSpeed);
        } else {
            Stop();
            return true;
        }
        return false;
    }

    public boolean init2() {
        // Code to move the elevator
        if (ClimbingMotor2.getPosition() < Math.round(NickClimbingConstanst.ClimbingMotorPoseition)) {
            ClimbingMotor2.set(NickClimbingConstanst.ClimbingSpeed);
        } else {
            Stop();
            return true;
        }
        return false;
    }

    public void Climb() {
        // Code to move the elevator
        ClimbingMotor1.set(-1);
        ClimbingMotor2.set(-1);
    }

    public void End() {
        ClimbingMotor1.stop();
        ClimbingMotor2.stop();
    }

    /**
     * Curently this is what we are using but after I get mesurments this should not
     * be used 1/28
     * 
     * @param Joy The joysitck that you are using
     */
    @Deprecated
    public void JoyClimb1(double Joy, boolean btuon) {
        ClimbingMotor1.set(Joy);
        if (btuon) {
            ClimbingMotor1.getEncoder().setPosition(0);
        }
    }

    @Deprecated
    public void JoyClimb2(double Joy, boolean btuon) {
        ClimbingMotor2.set(Joy);
        if (btuon) {
            ClimbingMotor2.getEncoder().setPosition(0);
        }
    }

    public void Stop() {
        // Code to stop the elevator
        ClimbingMotor1.stop();
        ClimbingMotor2.stop();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("CimbEndcoder 1", ClimbingMotor1.getPosition());
        SmartDashboard.putNumber("CimbEndcoder 2", ClimbingMotor2.getPosition());

    }

}
