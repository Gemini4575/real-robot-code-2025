package frc.robot.service;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveTrain;

public class DriveService {

    //report if one motor drives this much further than another for some reason
    private static double DRIFT_THRESHOLD = 0.2;

    private final DriveTrain driveTrain;

    private double distance;
    private double speed;
    private int driveMethod;
    private Pose2d startingPose;
    private double[] startingDriveValues;
    private boolean completed = false;
    private long startTime;
    private boolean drivingTriggered;

    public DriveService(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    public void startDriving(int driveMethod, double meters, double speed) {
        this.distance = meters;
        this.speed = Math.signum(distance) * speed;
        this.driveMethod = driveMethod;
        this.startingPose = driveTrain.getPose();
        this.startingDriveValues = getEncoderPositions();
        this.completed = false;
        this.startTime = System.currentTimeMillis();
        this.drivingTriggered = false;

        //actually start driving now
        doDrive();
    }

    private void doDrive() {
        switch(driveMethod) {
            case 1:
                driveTrain.driveRobotRelative(new ChassisSpeeds(speed, 0, 0));
                break;
            case 2:
                driveTrain.driveDirect(new ChassisSpeeds(speed, 0, 0));
                break;
            case 3:
                driveTrain.driveFieldRelative(new ChassisSpeeds(-speed, 0, 0));
                break;
            case 4:
                // if (System.currentTimeMillis() > startTime + 500) {
                //     if (!drivingTriggered) {
                //         driveTrain.driveViaController(1.05*distance);
                //         //drivingTriggered = true;
                //     }
                // } else {
                //     driveTrain.drive(speed, 0, 0, false, true);
                // }
                break;
            case 5:
                driveTrain.driveRobotRelative(new ChassisSpeeds(0, speed,0));
                break;
        }
    }

    public boolean keepDriving() {
        if (!completed) {
            var currentDistance = calculateDistance();
            SmartDashboard.putNumber("Distance driven", currentDistance);
            if (excessDistance(currentDistance) >= 0) {
                System.out.println("INFO: drive for " + distance + " meters is completed at " + currentDistance + " meters");
                driveTrain.stop();
                completed = true;
            } else {
                doDrive();
            }
        }
        return !completed;
    }

    private double[] getEncoderPositions() {
        var modulePositions = driveTrain.getModulePositions();
        var encPositions = new double[modulePositions.length];       
        for(int i =0; i < modulePositions.length; i++) {
            encPositions[i] = modulePositions[i].distanceMeters;
        }
        return encPositions;
    }

    private double calculateDistance() {
        var currentPose = driveTrain.getPose();
        var poseDistance = currentPose.getTranslation().getDistance(startingPose.getTranslation());
        if (excessDistance(poseDistance) >= 0) {
            System.out.println("WARN: According to estimated pose we drove " + poseDistance + " meters");
        }

        var currentDriveValues = getEncoderPositions();
        var encDistances = new ArrayList<Double>();
        for (int i = 0; i < currentDriveValues.length; i ++) {
            double oneDistance = Math.abs(currentDriveValues[i] - startingDriveValues[i]);
            SmartDashboard.putNumber("Module " + i + " distance", oneDistance);
            encDistances.add(oneDistance);
        }
        Collections.sort(encDistances);
        if (encDistances.get(encDistances.size()-1) -  encDistances.get(0) > DRIFT_THRESHOLD) {
            System.out.println("WARN: motor distances drifted apart " + encDistances);
        }
        var avgEncDistance = encDistances.stream().reduce(0.0, (x, y) -> x+y)/encDistances.size();
        if (Math.abs(poseDistance - avgEncDistance) > DRIFT_THRESHOLD) {
            System.out.println("WARN: motor distances vs pose distance drifed. PD " + poseDistance + " MD " + avgEncDistance);
        }
        return avgEncDistance;
    }

    private double excessDistance(double val) {
        return Math.abs(val) - Math.abs(distance);
    }
}
