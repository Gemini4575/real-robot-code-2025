// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.io.IOException;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

// import org.springframework.beans.factory.annotation.Autowired;
// import org.springframework.stereotype.Component;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.MesurementToRoation;
import frc.robot.Constants;



/** Represents a swerve drive style drivetrain. */
// @Component
public class DriveTrain extends SubsystemBase {
  private MesurementToRoation rotationsToInch = new MesurementToRoation();
  public boolean first;
  Field2d field = new Field2d();
  int ii = 0;
  public static final double kMaxAngularSpeed = 4.41 * 2 * Math.PI; // was Math.PI for 1/2 rotation per second
  
  double Ptranslate = 10.0;
  double Itranslate = 0.0;
  double Dtranslate = 0.0;
  double Protate = 15.0;
  double Irotate = 2.0;
  double Drotate = 0.0;
  
  SendableChooser<Pose2d> ahhhhhh;

private final Translation2d m_frontLeftLocation = Constants.SwerveConstants.m_frontLeftLocation;
private final Translation2d m_frontRightLocation = Constants.SwerveConstants.m_frontRightLocation;
private final Translation2d m_backLeftLocation = Constants.SwerveConstants.m_backLeftLocation;
private final Translation2d m_backRightLocation = Constants.SwerveConstants.m_backRightLocation;


private final SwerveModule m_backLeft = new SwerveModule(Constants.SwerveConstants.Mod0.constants);
private final SwerveModule m_backRight = new SwerveModule(Constants.SwerveConstants.Mod1.constants);
private final SwerveModule m_frontLeft = new SwerveModule(Constants.SwerveConstants.Mod3.constants);
private final SwerveModule m_frontRight = new SwerveModule(Constants.SwerveConstants.Mod2.constants);

//  private final Gyro_EPRA m_gyro = new Gyro_EPRA();
private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k100Hz);

private Double[] encoderDoubles = new Double[4];
private Double[] curencoderDoubles = new Double[4];
double startencoder = 0.0;
double target = 0.0;
double curencoder = 0.0;


private double xSpeed_cur;
private double ySpeed_cur;
private double rot_cur;
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          },
          new Pose2d(new Translation2d(),new Rotation2d(Units.degreesToRadians(180)))
          );

  private final SwerveDrivePoseEstimator poseEstimator;

  // private final DifferentialDrivetrainSim simDrive;
  // @Autowired
  // private MetricsProvider metricsProvider;

  public DriveTrain() {
    m_gyro.reset();

    
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | org.json.simple.parser.ParseException e) {
      e.printStackTrace();
      throw new RuntimeException(e);
    }

    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);
    poseEstimator =
            new SwerveDrivePoseEstimator(
                    m_kinematics,
                    m_gyro.getRotation2d(),
                    getModulePositions(),
                    new Pose2d(),
                    stateStdDevs,
                    visionStdDevs);

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(1, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(16.5, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    poseEstimator.resetPosition(new Rotation2d(180), getModulePositions(), new Pose2d(7.558, 4.010, new Rotation2d(180)));


  }
  
    public void ResetDrives () {
  
      /* 
      m_frontLeft.resetEncoder();
      m_frontRight.resetEncoder();
      m_backLeft.resetEncoder();
      m_backRight.resetEncoder();
      */
      m_gyro.reset();
  
      SmartDashboard.putString("Gyro has been reset", java.time.LocalTime.now().toString());
      System.out.println("Gyro has been reset");
    }
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    
    public void driveRobotRelative(ChassisSpeeds chassisSpeedsIn) {
      drive(chassisSpeedsIn.vxMetersPerSecond, chassisSpeedsIn.vyMetersPerSecond, chassisSpeedsIn.omegaRadiansPerSecond, false);
    }
    public void driveFieldRelative(ChassisSpeeds c) {
      drive(c.vxMetersPerSecond, c.vyMetersPerSecond, 0, true);
    }
    public void driveDirect(ChassisSpeeds chassisSpeedsIn) {
      var speeds = ChassisSpeeds.discretize(chassisSpeedsIn, LoggedRobot.defaultPeriodSecs);
      var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);
        m_frontLeft.setStateDirectly(swerveModuleStates[0]);
        m_frontRight.setStateDirectly(swerveModuleStates[1]);
        m_backLeft.setStateDirectly(swerveModuleStates[2]);
        m_backRight.setStateDirectly(swerveModuleStates[3]);
    }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(-1*xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

    SmartDashboard.putString("gyro", m_gyro.getRotation2d().toString());
    SmartDashboard.putString("module 0", swerveModuleStates[0].toString());
    SmartDashboard.putString("module 1", swerveModuleStates[1].toString());
    SmartDashboard.putString("module 2", swerveModuleStates[2].toString());
    SmartDashboard.putString("module 3", swerveModuleStates[3].toString());

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    Logger.recordOutput("SwerveStates", swerveModuleStates);

    //if(RobotState.isTest()) {


   // }
    xSpeed_cur = xSpeed;
    ySpeed_cur = ySpeed;
    rot_cur = rot;
  }

  public void stop() {
    drive(-0, -0, -0, false);
  }

  public ChassisSpeeds getSpeed() {
    return new ChassisSpeeds(xSpeed_cur, ySpeed_cur, rot_cur);
  }

  private ChassisSpeeds getRobotRelativeSpeeds(){
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates(){
      return new SwerveModuleState[]{
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_backLeft.getState(),
          m_backRight.getState()
      };
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        getModulePositions());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  } 

  public void resetPose(Pose2d aPose2d) {
   m_odometry.resetPosition(m_gyro.getRotation2d(),
        getModulePositions(), aPose2d );
    poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), aPose2d);
  }
  
  /* adding some gyro output so other classes don't need to access it directly */
  public double getYaw() {
    return (m_gyro.getAngle());
  }

  public Rotation2d getFacing() {
    return m_gyro.getRotation2d();
  }

  

    public PathConstraints getChassisConstrains() {
        return new PathConstraints(
          3.000,
          3.000,
          Units.degreesToRadians(540.000),
          Units.degreesToRadians(720.000)
        );
    }
    
    public boolean DriveMeters(double meters) {
      if(first) {
        first = false;
        encoderDoubles[0] = m_frontLeft.getEncoderValue();
        encoderDoubles[1] = m_frontRight.getEncoderValue();
        encoderDoubles[2] = m_backLeft.getEncoderValue();
        encoderDoubles[3] = m_backRight.getEncoderValue();
        startencoder = Math.abs(java.util.Arrays.stream(encoderDoubles).mapToDouble(Double::doubleValue).average().orElse(0.0));
      }
      curencoder = Math.abs(java.util.Arrays.stream(curencoderDoubles).mapToDouble(Double::doubleValue).average().orElse(0.0));
      target = Math.abs((startencoder + rotationsToInch.calculateRotationsM(meters, 6.75, 6)) - java.util.Arrays.stream(curencoderDoubles).mapToDouble(Double::doubleValue).average().orElse(0.0));
      double remainingDistance = target - curencoder;
      if (Math.abs(Math.round(remainingDistance)) <= 0) {
        stop();
        return true;
      } else {
        drive(0, 0.1, 0, false);
        SmartDashboard.putNumber("Curencoder", curencoder);
        SmartDashboard.putNumber("encoder", startencoder);
        SmartDashboard.putNumber("target", target);
        return false;
      }
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      poseEstimator.update(m_gyro.getRotation2d(), getModulePositions());
      field.setRobotPose(poseEstimator.getEstimatedPosition());
    
      SmartDashboard.putData("robotpose", field);

      SmartDashboard.putNumber("Gyro yaw", m_gyro.getYaw());
      SmartDashboard.putNumber("Gyro pitch", m_gyro.getPitch());
      SmartDashboard.putNumber("Gyro roll", m_gyro.getRoll());
      SmartDashboard.putNumber("Gyro angle", m_gyro.getAngle());

      curencoderDoubles[0] = m_frontLeft.getEncoderValue();
      curencoderDoubles[1] = m_frontRight.getEncoderValue();
      curencoderDoubles[2] = m_backLeft.getEncoderValue();
      curencoderDoubles[3] = m_backRight.getEncoderValue();
      
      /*super.simulationPeriodic();

      // this is probably not the right speed calculation.. need to figure out where to get the speed
      m_frontLeft.getDriveMotorSim().iterate(xSpeed_cur + ySpeed_cur, m_frontLeft.getDriveMotorSim().getBusVoltage(), 0.02);
      m_frontRight.getDriveMotorSim().iterate(xSpeed_cur + ySpeed_cur, m_frontRight.getDriveMotorSim().getBusVoltage(), 0.02);

      simDrive.setInputs(m_frontLeft.getDriveMotorSim().getAppliedOutput() * m_frontLeft.getDriveMotorSim().getBusVoltage(),  m_frontRight.getDriveMotorSim().getAppliedOutput() * m_frontRight.getDriveMotorSim().getBusVoltage());
      simDrive.update(0.02);
      
      metricsProvider.updateLocation(simDrive.getPose());*/
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }
}
