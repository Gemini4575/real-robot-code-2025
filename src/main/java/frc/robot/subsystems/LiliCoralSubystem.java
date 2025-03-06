package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.LiliCoralConstants;
import frc.robot.Constants.RobotMode;

public class LiliCoralSubystem extends SubsystemBase{

    private static enum State {OPEN, CLOSED, CLOSING, OPENING}

    SparkMax gate;
    //DigitalInput top;
    DigitalInput bottom;
    DigitalInput coral;
    Timer timer = new Timer();

    private State lastKnownState = State.CLOSED;

    // 0.540528
    public LiliCoralSubystem(){
        gate = new SparkMax(LiliCoralConstants.CoarlMotor, MotorType.kBrushed);
    //    top = new DigitalInput(LiliCoralConstants.Top);
        bottom = new DigitalInput(LiliCoralConstants.Top);
        coral = new DigitalInput(LiliCoralConstants.Coral);
        lastKnownState = State.CLOSED;
        first = true;
    }

    // private boolean top() {
    //     return top.get();
    // }

    private boolean bottom() {
        return bottom.get();
    }

    private boolean coral() {
        return coral.get();
    }

    // public boolean intakeCoral() {
    //     if ((lastKnownState == State.CLOSED || lastKnownState == State.CLOSING) && bottom()) {
    //         gate.set(0);
    //         lastKnownState = State.CLOSED;
    //         return coral();
    //     }
    //     gate.set(LiliCoralConstants.GateSpeed);
    //     lastKnownState = State.CLOSING;
    //     return false;
    // }

    // public boolean placeCoral() {
    //     if(!coral()) {
    //         gate.set(0);
    //         return true;
    //     }
    //     if ((lastKnownState == State.OPEN || lastKnownState == State.OPENING) && bottom()) {
    //         gate.set(0);
    //         lastKnownState = State.OPEN;
    //     } else {
    //         gate.set(-LiliCoralConstants.GateSpeed);
    //         lastKnownState = State.OPENING;
    //     }
    //     return false;
    // }

    // public boolean DropGate() {
    //     if(bottom() && lastKnownState == State.CLOSED) {
    //         gate.set(LiliCoralConstants.GateSpeed *-1);
    //     } else {
    //         lastKnownState = State.OPEN;
    //         gate.set(0);
    //     }
    //     return coral();
    // }

    // public boolean CloseGate() {
    //     if(bottom() && lastKnownState == State.OPEN) {
    //         gate.set(LiliCoralConstants.GateSpeed);
    //     } else {
    //         lastKnownState = State.CLOSED;
    //         gate.set(0);
    //         return true;
    //     }
    //     return false;
    // }
    boolean pervois = false;
    boolean first = true;
    private void start() {
        if(first) {
            pervois = true;
            first = false;
        }
    }
    public void isInterupted() {
        pervois = false;
        first = true;
    }
    
    public boolean DropGate() {
        start();
        if(!pervois && bottom()) {
            gate.stopMotor();
            first = true;
            return true;
        } else {
            gate.set(LiliCoralConstants.GateSpeed);
            pervois = bottom();
        }

        return false;//!coral();
    }

    public boolean CloseGate() {
        start();
        if((!pervois && bottom()) || coral()) {
            gate.stopMotor();
            first = true;
            return true;
        } else {
            gate.set(-LiliCoralConstants.GateSpeed);
            pervois = bottom();
            return false;
        }
    }

    public boolean GetCoral() {
        if((pervois && bottom()) || coral()) {
            gate.stopMotor();
            first = true;
            return true;
        } else {
            gate.set(-LiliCoralConstants.GateSpeed);
            pervois = bottom();
            return false;
        }
    }

    public boolean CloseGateSlow() {
        start();
        if((!pervois && bottom()) || coral()) {
            gate.stopMotor();
            first = true;
            return true;
        } else {
            gate.set(-LiliCoralConstants.GateSpeed/2.0);
            pervois = bottom();
            return false;
        }
    }

    
    /**
     * SHOULD NOT BE USED IN COMP
     * @param joy the joystick axis your using
     */
    @Deprecated
    public void JoyControll(double joy) {
        gate.set(joy);
    }

    public BooleanSupplier Coral() {
        return () -> coral();
    }
    

    @Override
    public void periodic() {
        if((DriverStation.isFMSAttached() || Robot.IS_COMPETITION) || RobotState.isTest()) {
        SmartDashboard.putBoolean("Gate", bottom());
        SmartDashboard.putBoolean("Coral", coral());
        }
        if(first) {
            if(bottom() && lastKnownState == State.CLOSED) {
                
            }
        }
    }

}
