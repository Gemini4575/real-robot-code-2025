package frc.robot.subsystems;


import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PoseTools;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    Vision vision;
    boolean InRange = false;
    Rotation2d targetYaw;
    double targetRange = 0.0;
    private PoseTools translation3dToTranslation2d;

    public VisionSubsystem(Vision vision) {
        this.vision = vision;
    }

    @Override
    public void periodic() {
        // publish relative algae position
        var algae = vision.getAlgaeTarget();
        if (algae != null) {
            SmartDashboard.putString("Algae X translation", algae.getBestCameraToTarget().getMeasureX().toString());
            SmartDashboard.putString("Algae Y translation", algae.getBestCameraToTarget().getMeasureY().toString());
        }
    }

    public boolean InRange() {
        var results = vision.getTagCamera().getAllUnreadResults();

        if (!results.isEmpty()) {
            if (translation3dToTranslation2d
                    .calculate(
                            vision.getTagCamera().getLatestResult().getBestTarget().bestCameraToTarget.getTranslation())
                    .getDistance(new Translation2d(0.127, 0.127)) > 0.2) {
                InRange = true;
            } else {
                InRange = false;
            }
            return InRange;
        }
        return false;
    }

    public double getXSpeed(int side) {
        ChassisSpeeds speeds = processImage(side);
        if (speeds != null) {
            return speeds.vxMetersPerSecond;
        } else {
            return 0.0;
        }
    }

    public double getYSpeed(int side) {
        ChassisSpeeds speeds = processImage(side);
        if (speeds != null) {
            return speeds.vyMetersPerSecond;
        } else {
            return 0.0;
        }
    }

    public double getOmegaSpeed(int side) {
        ChassisSpeeds speeds = processImage(side);
        if (speeds != null) {
            return speeds.omegaRadiansPerSecond;
        } else {
            return 0.0;
        }
    }

    public int getFiducialId() {
        return vision.getTagCamera().getLatestResult().getBestTarget().fiducialId;
    }

    public ChassisSpeeds processImage(int Side) {
        var results = vision.getTagCamera().getAllUnreadResults();

        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == Side) {
                        Pose3d robotpose = vision.getEstimatedGlobalPose().get().estimatedPose;
                        Pose3d targetPose = Constants.Vision.kTagLayout.getTagPose(Side).get();
                        targetYaw = PhotonUtils.getYawToPose(robotpose.toPose2d(), targetPose.toPose2d());
                        targetRange = PhotonUtils.getDistanceToPose(robotpose.toPose2d(), targetPose.toPose2d());
                        SmartDashboard.putNumber("fowrad", (targetRange * Math.cos(targetYaw.getRadians())));
                        SmartDashboard.putNumber("Strafe", targetRange * Math.sin(targetYaw.getRadians()));
                        ChassisSpeeds speeds = new ChassisSpeeds(
                                targetRange * Math.cos(targetYaw.getRadians()),
                                targetRange * Math.sin(targetYaw.getRadians()),
                                0);
                        return speeds;
                    }
                }
            }
        }

        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
        return speeds;
    }

    public void getAlgaeTarget() {
        //Field2d field = new Field2d();
        var algae = vision.getAlgaeTarget();
         if (algae != null) {
            SmartDashboard.putNumber("Algae pitch", algae.getPitch());
            SmartDashboard.putNumber("Algae skew", algae.getSkew());
            SmartDashboard.putNumber("Algae yaw", algae.getYaw());
            SmartDashboard.putNumber("Algae area", algae.getArea());
            //var translation = algae.getBestCameraToTarget().getTranslation();
            //Pose2d algaePose = new Pose2d(translation.getX(), translation.getY(), new Rotation2d());
            //field.setRobotPose(algaePose);
            //SmartDashboard.putData("Algae Pose", field);
        }
    }
}