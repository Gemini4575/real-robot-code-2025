package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.epramotor.Motor;
import frc.robot.Constants.NickClimbingConstanst;

public class NickClimbingSubsystem extends SubsystemBase{
    
    public Motor ClimbingMotor1;
    public Motor ClimbingMotor2;

    public NickClimbingSubsystem() {
        // Initialization code here
        ClimbingMotor1 = new Motor(NickClimbingConstanst.ClimbingMotor1, MotorType.kBrushless);
        ClimbingMotor2 = new Motor(NickClimbingConstanst.ClimbingMotor2, MotorType.kBrushless);
    }

    private boolean Climb1() {
        // Code to move the elevator
        return ClimbingMotor1.SetIfBoolean(ClimbingMotor1.getPosition() < (NickClimbingConstanst.ClimbingMotorPoseition), NickClimbingConstanst.ClimbingSpeed);
    }
    
    private boolean Climb2() {
        // Code to move the elevator
        return ClimbingMotor2.SetIfBoolean((ClimbingMotor2.getPosition()) < (NickClimbingConstanst.ClimbingMotorPoseition), NickClimbingConstanst.ClimbingSpeed);
    }

    public boolean init1() {
        // Code to move the elevator
        if(ClimbingMotor1.getPosition() < Math.round(0)) {
            ClimbingMotor1.set(NickClimbingConstanst.ClimbingSpeed);
            } else {
                Stop();
            return true;
            }
            return false;
    }

    public boolean init2() {
        // Code to move the elevator
        if(ClimbingMotor2.getPosition() < Math.round(NickClimbingConstanst.ClimbingMotorPoseition)) {
            ClimbingMotor2.set(NickClimbingConstanst.ClimbingSpeed);
            } else {
                Stop();
            return true;
            }
            return false;
    }
    public boolean Climb() {
        // Code to move the elevator
        if(Climb1() || Climb2()) {
            Stop();
            return true;
        }
        return false;
    }
    /**
     * Curently this is what we are using but after I get mesurments this should not be used 1/28
     * @param Joy The joysitck that you are using
     */
    @Deprecated
    public void JoyClimb(double Joy) {
        ClimbingMotor1.set(Joy);
        ClimbingMotor2.set(Joy);
    }
    
    public void Stop() {
        // Code to stop the elevator
        ClimbingMotor1.stop();
        ClimbingMotor2.stop();
    }

}
