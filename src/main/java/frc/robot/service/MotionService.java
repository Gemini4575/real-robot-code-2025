package frc.robot.service;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TelopSwerve;
import frc.robot.commands.coral.lili.EXOCloseGateSlow;
import frc.robot.commands.coral.lili.EXODropGate;
import frc.robot.commands.coral.lili.EXOGetCoral;
import frc.robot.commands.coral.lili.EXOOpenGate;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.drive.DriveTwoardsAprillTag;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.drive.Strafe;
import frc.robot.commands.drive.Turn;
import frc.robot.datamodel.MotionDirective;
import frc.robot.datamodel.MotionDirective.MotionType;
import frc.robot.subsystems.LiliCoralSubystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DriveTrain;

public class MotionService {

    private final DriveTrain driveTrain;
    private final LiliCoralSubystem c;
    private final VisionSubsystem v;

    private MotionDirective[] motions;
    private int currentStep = -1;
    private Command currentCommand;

    public MotionService(DriveTrain driveTrain, LiliCoralSubystem c, VisionSubsystem v) {
        this.driveTrain = driveTrain;
        this.c = c;
        this.v = v;
    }

    public synchronized void executeSequence(MotionDirective... motions) {
        this.motions = motions;
        this.currentStep = 0;
        SmartDashboard.putString("Motion Status", "Starting");
        startMotion();
    }

    public synchronized void stop(boolean force) {
        SmartDashboard.putString("Motion Status", "Stopping");
        if (force && currentCommand != null) {
            currentCommand.end(true);
        }
        motions = null;
        currentStep = -1;
    }

    public synchronized void periodic() {
        if (motions != null && currentStep >=0 && currentCommand != null && currentCommand.isFinished()) {
            if(motions[currentStep].getType() == MotionType.DROP_CORAL) {
                new EXOOpenGate(c).asProxy().schedule();
            }
            if (currentStep < motions.length-1) {
                currentStep++;
                startMotion();
            } else {
                stop(false);
            }
        }
    }

    private void startMotion() {
        switch (motions[currentStep].getType()) {
            case DRIVE:
                currentCommand = new DriveStraight(driveTrain, motions[currentStep].getAmount());
                break;
            case TURN:
                currentCommand = new Turn(driveTrain, motions[currentStep].getAmount());
                break;
            case DROP_CORAL:
                currentCommand = new EXODropGate(c).withTimeout(1);
                break;
            case STRAFE:
                currentCommand = new Strafe(driveTrain, motions[currentStep].getAmount());
                break;
            case WAIT:
                currentCommand = new WaitCommand(motions[currentStep].getAmount());
                break;
            case STOP:
                currentCommand = new Stop(driveTrain);
                break;
            case GET_CORAL:
                currentCommand = new EXOCloseGateSlow(c).withTimeout(1);
                break;
            case APRILTAG_DRIVE:
                //.currentCommand = new DriveTwoardsAprillTag(v, driveTrain);
                break;
            case CAMERA_DRIVE:
                double driveDistance = v.getDistanceToDrive();
                currentCommand = new DriveStraight(driveTrain, driveDistance);
                break;
            case CAMERA_STRAFE:
                double strafeDistance = v.getStrafeDistance();
                currentCommand = new Strafe(driveTrain, strafeDistance);
                break;
            case CAMERA_ROTATE:
                double rotationAngle = v.getRotationAngle();
                currentCommand = new Turn(driveTrain, rotationAngle);
                break;
            case CAMERA_ALL:
                double[] motionParams = v.getAllMotionParameters();
                currentCommand = new TelopSwerve(driveTrain,
                    () -> motionParams[1], 
                    () -> motionParams[2], 
                    () -> motionParams[3], 
                    () -> false);
                break;
        }
        publshStartStatus();
        currentCommand.schedule();
    }

    private void publshStartStatus() {
        SmartDashboard.putString("Motion Index", String.valueOf(currentStep));
        SmartDashboard.putString("Starting motion", motions[currentStep].toString());
    }

}
