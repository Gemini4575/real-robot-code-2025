package frc.robot;


import static frc.robot.datamodel.MotionDirective.GetCoral;
import static frc.robot.datamodel.MotionDirective.drive;
import static frc.robot.datamodel.MotionDirective.dropCoral;
import static frc.robot.datamodel.MotionDirective.stop;
import static frc.robot.datamodel.MotionDirective.strafe;
import static frc.robot.datamodel.MotionDirective.turn;
import static frc.robot.datamodel.MotionDirective.wait2;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.math.MesurementToRoation;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.datamodel.MotionDirective;

public class Constants {
    public static MesurementToRoation rotationsToInch = new MesurementToRoation();

    public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

    public static final String ROBOT_NAME = "4575-2025";

    // avoid typo errors
    public static final class LogConfigs {
        public static final String
                SYSTEM_PERFORMANCE_PATH = "SystemPerformance/",
                PHYSICS_SIMULATION_PATH = "MaplePhysicsSimulation/",
                APRIL_TAGS_VISION_PATH = "Vision/AprilTags/",
                SHOOTER_PATH = "Shooter/";
    }

    public static final class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class Reef {
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d RedBargeSidePassthough = new Pose2d(
        4.496,
        1.264,
        Rotation2d.fromDegrees(0));
    public static final Pose2d BlueBargeSidePassthough = new Pose2d(
        4.475,
        6.796,
        Rotation2d.fromDegrees(0));
    

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefHeight, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

    static {
      // Initialize faces
    //   centerFaces[-1] =
    //       new Pose2d(
    //           Units.inchesToMeters(144.003),
    //           Units.inchesToMeters(158.500),
    //           Rotation2d.fromDegrees(180));
      centerFaces[0] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      centerFaces[1] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      centerFaces[2] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      centerFaces[3] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      centerFaces[4] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      centerFaces[5] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
        Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
        for (var level : ReefHeight.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
        }
        // branchPositions.add((face * 2) + 1, fillRight);
        // branchPositions.add((face * 2) + 2, fillLeft);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefHeight(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public final double height;
    public final double pitch;
  }
}

    public static Rotation2d toCurrentAllianceRotation(Rotation2d rotationAtBlueSide) {
        final Rotation2d
                yAxis = Rotation2d.fromDegrees(90),
                differenceFromYAxisAtBlueSide = rotationAtBlueSide.minus(yAxis),
                differenceFromYAxisNew = differenceFromYAxisAtBlueSide.times(isSidePresentedAsRed() ? -1:1);
        return yAxis.rotateBy(differenceFromYAxisNew);
    }

    public static Translation2d toCurrentAllianceTranslation(Translation2d translationAtBlueSide) {
        if (isSidePresentedAsRed())
            return new Translation2d(
                    FieldConstants.fieldWidth - translationAtBlueSide.getX(),
                    translationAtBlueSide.getY()
            );
        return translationAtBlueSide;
    }

    public static Translation3d toCurrentAllianceTranslation(Translation3d translation3dAtBlueSide) {
        final Translation2d translation3dAtCurrentAlliance = toCurrentAllianceTranslation(translation3dAtBlueSide.toTranslation2d());
        if (isSidePresentedAsRed())
            return new Translation3d(
                    translation3dAtCurrentAlliance.getX(),
                    translation3dAtCurrentAlliance.getY(),
                    translation3dAtBlueSide.getZ()
            );
        return translation3dAtBlueSide;
    }

    public static Pose2d toCurrentAlliancePose(Pose2d poseAtBlueSide) {
        return new Pose2d(
                toCurrentAllianceTranslation(poseAtBlueSide.getTranslation()),
                toCurrentAllianceRotation(poseAtBlueSide.getRotation())
        );
    }

    public static PathPlannerPath toCurrentAlliancePath(PathPlannerPath pathAtBlueAlliance) {
        return isSidePresentedAsRed() ? pathAtBlueAlliance.flipPath() : pathAtBlueAlliance;
    }

    public static boolean isSidePresentedAsRed() {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get().equals(DriverStation.Alliance.Red);
    }

    public static Rotation2d getDriverStationFacing() {
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
            case Red -> new Rotation2d(Math.PI);
            case Blue -> new Rotation2d(0);
        };
    }
    public static final double stickDeadband = 0.2;
    
    public static final class Vision {
        public static final String kTagCameraName = "Arducam1";
        public static final String kTagCameraColorName = "Arducam IMX179 Camera Module";
        public static final String kAlgaeCameraName = "ArducamColor";
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
        //TODO update with real value
        public static final Transform3d kRobotToCam =
                new Transform3d(Units.inchesToMeters(1.25), Units.inchesToMeters(10.5), Units.inchesToMeters(17.75), new Rotation3d(0, 0, Math.PI/2));

        public static final Transform3d kRobotToCamColor =
                new Transform3d(Units.inchesToMeters(1.5), -Units.inchesToMeters(6.5), Units.inchesToMeters(17.25), new Rotation3d(0, 0, -Math.PI/2));
                // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        private static final int ALGAE_CAM_WIDTH = 1200;
        private static final int ALGAE_CAM_HEIGHT = 800;
        public static final int ALGAE_CAM_AREA = ALGAE_CAM_HEIGHT * ALGAE_CAM_WIDTH;
    
        public static final double ALGAE_CAM_FOCAL_LENGTH = 70;
    
        public static final double ALGAE_REAL_DIAMETER = Units.inchesToMeters(16.25); //meters
    }

    public static final class SwerveConstants {
         // these are distance from the center to the wheel in meters. .381 is 1.25 feet or 16 inches
        // swerve drive has 35.5 inch diagonals
/*
        private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
*/
/*  These numbers are for 28.5 swerve 
public static final Translation2d m_frontLeftLocation = new Translation2d(0.4445, 0.4445);
public static final Translation2d m_frontRightLocation = new Translation2d(0.4445, -0.4445);
public static final Translation2d m_backLeftLocation = new Translation2d(-0.4445, 0.4445);
public static final Translation2d m_backRightLocation = new Translation2d(-0.4445, -0.4445);

/*  These numbers are for 29.5 swerve
0.45085
private final Translation2d m_frontLeftLocation = new Translation2d(0.45085, 0.45085);
private final Translation2d m_frontRightLocation = new Translation2d(0.45085, -0.45085);
private final Translation2d m_backLeftLocation = new Translation2d(-0.45085, 0.45085);
private final Translation2d m_backRightLocation = new Translation2d(-0.45085, -0.45085);

//These numbers are for the weird rectangle swerve
//0.2032 X
//0.2794 Y
        public static final Translation2d m_frontLeftLocation = new Translation2d(0.2032, 0.2794);
        public static final Translation2d m_frontRightLocation = new Translation2d(0.2032, -0.2794);
        public static final Translation2d m_backLeftLocation = new Translation2d(-0.2032, 0.2794);
        public static final Translation2d m_backRightLocation = new Translation2d(-0.2032, -0.2794);
        */
        // These number are for a 25 by 25 swerve
        // 0.33333
        private static final double ROBOT_WIDTH = Units.inchesToMeters(25.0);
        private static final double ROBOT_LENGTH = Units.inchesToMeters(25.0);
        private static final double SWERVE_FROM_CORNER = Units.inchesToMeters(2.61);
        private static final double MODULE_OFFSET_X = ROBOT_WIDTH/2 - SWERVE_FROM_CORNER;
        private static final double MODULE_OFFSET_Y = ROBOT_LENGTH/2 - SWERVE_FROM_CORNER;
        public static final Translation2d m_backLeftLocation = new Translation2d(-MODULE_OFFSET_X, MODULE_OFFSET_Y);
        public static final Translation2d m_backRightLocation = new Translation2d(-MODULE_OFFSET_X, -MODULE_OFFSET_Y);
        public static final Translation2d m_frontRightLocation = new Translation2d(MODULE_OFFSET_X, -MODULE_OFFSET_Y);
        public static final Translation2d m_frontLeftLocation = new Translation2d(MODULE_OFFSET_X, MODULE_OFFSET_Y);

        /* Ints */
            public static final int kEncoderResolution = 4096;
        /* Doubles */
            public static final double MaxMetersPersecond = 4.47;//3.264903459; //4.47 This is calculated 5676rpm, 4in wheels, 6.75 gearbox
            public static final double kWheelRadius = 0.0508;
            public static final double kModuleMaxAngularVelocity =  27.73816874; //This is calculated 5676rpm, 150/7:1 gearbox in radians. 594.380 deg/s in pathplanner
            public static final double kModuleMaxAngularAcceleration = 18.85;//4 * Math.PI; // radians per second squared
            // FWF - stole this from 6328's code, each gear reduction written out. Final is 6.75. 39.37 converts inches to meters so we can be european fancy
            //private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
            //public static final double driveAfterEncoderReduction = (4.0 / 39.37) * Math.PI * (1/6.75);
//            public static final double driveAfterEncoderReduction = 0.0788114854;   // FWF this is the above calc * 1.6667 to see if the auto distance changes


            public static final double driveConversionFactor = (Math.PI  + 2.0 * kWheelRadius)/6.75;
            public static final double turnAfterEncoderReduction = -1 * (7/150);
            public static final double gearboxRatio = 6.75;
            /**
             * In meters
             */
            public static final double wheeldiameter = Units.inchesToMeters(4.0);
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 0;
            public static final double angleOffset = 3.201315307;
            public static final double speedAdjustmentFactor = 1;//1.798006206333298/4.0;//2.092980946810132;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, speedAdjustmentFactor);
        }

        /** Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 1;
            public static final double angleOffset = 5.333449307;
            public static final double speedAdjustmentFactor = 1;//1.798006206333298/1.891452461749773;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, speedAdjustmentFactor);
        }
        
        /** Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 2;
            public static final double angleOffset = 0.2082833072;
            public static final double speedAdjustmentFactor = 1;//1.798006206333298/1.8846972405721;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, speedAdjustmentFactor);
        }

        /** Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 3;
            public static final double angleOffset = 3.769512307;
            public static final double speedAdjustmentFactor = 1;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, speedAdjustmentFactor);
        }

        public static final double one_meter = rotationsToInch.calculateRotationsM(1, 1,1);
    }

    public final static class JoystickConstants{
        public final static int DRIVER_USB = 0;
        public final static int OPERATOR_USB = 1;
        public final static int TEST_USB = 2;
        
        public final static int LEFT_Y_AXIS = 1;
        public final static int LEFT_X_AXIS = 0;
        public final static int RIGHT_X_AXIS = 4;
        public final static int RIGHT_Y_AXIS = 5;
    
        public final static int GREEN_BUTTON = 1;
        public final static int RED_BUTTON = 2;
        public final static int YELLOW_BUTTON = 4;
        public final static int BLUE_BUTTON = 3;
    
        public final static int LEFT_TRIGGER = 2;
        public final static int RIGHT_TRIGGER = 3;
        public final static int LEFT_BUMPER = 5;
        public final static int RIGHT_BUMPER = 6;
    
        public final static int BACK_BUTTON = 7;
        public final static int START_BUTTON = 8;
    
        public final static int POV_UP = 0;
        public final static int POV_RIGHT = 90;
        public final static int POV_DOWN = 180;
        public final static int POV_LEFT = 270;
    }

    public final static class ArmConstants {
        /* Ints */
            public final static int ARM_MOTOR = 15;
            public final static int TOP_Wheel = 0;
            public final static int BOTTOM_Wheel = 1;
            public final static int CoralSensor = 0;
        /* Doubles */
            public final static double L1Position = 1;
            public final static double L2Position = 1.0;
            public final static double L3Position = 122;
            public final static double L4Position = 0.0;
            public final static double CoralStationPosition = 54.0;
            public final static double maxSpeed = 0.0;
            public final static double maxAcceleration = 0.0;
            public final static double IntakeSpeed = 0.5;
            
    }

    public final static class NickClimbingConstanst {
        /* Ints */
            public final static int ClimbingMotor1 = 14;
            public final static int ClimbingMotor2 = 13;
            public final static double ClimbingMotorPoseition = 65;
        /* Doubles */
            public final static double ClimbingSpeed = 0.25;
    }

    public final static class OzzyGrabberConstants {
        /* Ints */
            public final static int GrabberMotor = 16;
            public final static int PosetionMotor = 17;
            public final static int BeamBreak = 8;
            public final static int Bottom = 3;
            public final static int top = 4;
        /* Doubles */
            public final static double IntakeSpeed = 0.8;
            public final static double OutakeSpeed = -0.8;
            public final static double UpSpeed = 0.9;
            public final static double DownSpeed = 0.8;
            public final static double MovmentLength = 12.5;
            public final static double MiddleLength = 0.0;
    }

    public final static class LiliCoralConstants {
        /* Ints */
            public final static int CoarlMotor = 18;
            public final static int Coral = 1;
            public final static int Top = 2;
            public final static int Bottom = 3;
        /* Doubles */
            public final static double GateSpeed = -1;
    }

    public final static class  Autos {
        public static final MotionDirective[] AUTO_WITH_CAM = new MotionDirective[]{strafe(82), dropCoral(), stop(), GetCoral(), drive(-200), strafe(196)};
        public static final MotionDirective[] AUTO_CORAL1 = new MotionDirective[]{drive(68.0), turn(80), dropCoral(), stop()};
        public static final MotionDirective[] AUTO_CORAL2 = new MotionDirective[]{drive(79.0), turn(80), dropCoral(), stop(),
        GetCoral(), drive(-95), turn(-85), drive(170), strafe(-30), turn(-25), turn(75), strafe(35), 
        drive(25), wait2(1), GetCoral(), turn(180), strafe(175), dropCoral(), stop(), GetCoral()};//]\[], turn(-10), drive(-Units.inchesToMeters(90)), strafe(Units.inchesToMeters(230))};
        
    }

    public final static class FieldLocations {
        public static Pose2d ALGAE_INTAKE = new Pose2d(6, 0.45, Rotation2d.fromDegrees(90));
    }

}