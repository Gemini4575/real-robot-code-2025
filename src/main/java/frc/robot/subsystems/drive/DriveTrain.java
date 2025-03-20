// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.io.IOException;

import javax.xml.crypto.dsig.keyinfo.KeyValue;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

// import org.springframework.beans.factory.annotation.Autowired;
// import org.springframework.stereotype.Component;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
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
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

import edu.wpi.first.wpilibj.RobotController;

/** Represents a swerve drive style drivetrain. */
// @Component
public class DriveTrain extends SubsystemBase {
  public boolean first;
  Field2d field = new Field2d();
  int ii = 0;
  public static final double kMaxSpeed = SwerveConstants.MaxMetersPersecond; // was 4.47 meters per second
  public static final double kMaxAngularSpeed = 4.41 * 2 * Math.PI; // was Math.PI for 1/2 rotation per second
  
  double Ptranslate = 10.0;
  double Itranslate = 0.0;
  double Dtranslate = 0.0;
  double Protate = 15.0;
  double Irotate = 2.0;
  double Drotate = 0.0;
  
  SendableChooser<Pose2d> ahhhhhh;

private final Translation2d m_backLeftLocation = Constants.SwerveConstants.m_backLeftLocation;
private final Translation2d m_backRightLocation = Constants.SwerveConstants.m_backRightLocation;
private final Translation2d m_frontRightLocation = Constants.SwerveConstants.m_frontRightLocation;
private final Translation2d m_frontLeftLocation = Constants.SwerveConstants.m_frontLeftLocation;


private final SwerveModule m_backLeft_0 = new SwerveModule(Constants.SwerveConstants.Mod0.constants);
private final SwerveModule m_backRight_1 = new SwerveModule(Constants.SwerveConstants.Mod1.constants);
private final SwerveModule m_frontRight_2 = new SwerveModule(Constants.SwerveConstants.Mod2.constants);
private final SwerveModule m_frontLeft_3 = new SwerveModule(Constants.SwerveConstants.Mod3.constants);

//  private final Gyro_EPRA m_gyro = new Gyro_EPRA()
private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k100Hz);

// private Double[] encoderDoubles = new Double[4];
// private Double[] curencoderDoubles = new Double[4];
double startencoder = 0.0;
double target = 0.0;
double curencoder = 0.0;


private double xSpeed_cur;
private double ySpeed_cur;
private double rot_cur;
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_backLeftLocation, m_backRightLocation, m_frontRightLocation, m_frontLeftLocation);
          //m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation); //Changed the order

  // private final SwerveDriveOdometry m_odometry =
  //     new SwerveDriveOdometry(
  //         m_kinematics,
  //         m_gyro.getRotation2d(),
  //         new SwerveModulePosition[] {
  //           m_frontLeft.getPosition(),
  //           m_frontRight.getPosition(),
  //           m_backLeft.getPosition(),
  //           m_backRight.getPosition()
  //         },
  //         new Pose2d(new Translation2d(),new Rotation2d(Units.degreesToRadians(180)))
  //         );

  private final SwerveDrivePoseEstimator poseEstimator;

  // private final DifferentialDrivetrainSim simDrive;
  // @Autowired
  // private MetricsProvider metricsProvider;

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;
  private RobotConfig config;

  public DriveTrain() {
    m_gyro.reset();
    
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

    poseEstimator.resetPosition(new Rotation2d(180), getModulePositions(), new Pose2d(7.558, 4.010, new Rotation2d(180)));

    configureAutoBuilder();

    setpointGenerator = new SwerveSetpointGenerator(
      config,
      Units.rotationsToRadians(1.0) // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
    );
    previousSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(config.numModules));
  }

  public void configureAutoBuilder() {
    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveForPathPlanner(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(6, 0, 0.0), // Translation PID constants
                    new PIDConstants(1.5, 0.0, 0.0) // Rotation PID constants
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
  }
  
  public void driveForPathPlanner(ChassisSpeeds speeds) {
    SmartDashboard.putString("[Drivetrain]ChassisSpeeds", speeds.toString());
    // Note: it is important to not discretize speeds before or after
    // using the setpoint generator, as it will discretize them for you
    previousSetpoint = setpointGenerator.generateSetpoint(
        previousSetpoint, // The previous setpoint
        speeds, // The desired target speeds
        0.02 // The loop time of the robot code, in seconds
    );
    SmartDashboard.putString ("[Drivetrain]Previous Setpoint", previousSetpoint.toString());
    setModuleStates(previousSetpoint.moduleStates()); // Method that will drive the robot given target module states
  }

    public void ResetDrives () {
  
      /* 
      m_frontLeft.resetEncoder();
      m_frontRight.resetEncoder();
      m_backLeft.resetEncoder();
      m_backRight.resetEncoder();
      */
      m_gyro.reset();
  
      SmartDashboard.putString("[Drivetrain]Gyro has been reset", java.time.LocalTime.now().toString());
      System.out.println("Gyro has been reset");
      Logger.recordOutput("Gyro Has Been reset", DriverStation.getMatchTime());
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
      SmartDashboard.putString("[Drivetrain]ChassisSpeeds", chassisSpeedsIn.toString());
      drive(chassisSpeedsIn.vxMetersPerSecond, chassisSpeedsIn.vyMetersPerSecond, chassisSpeedsIn.omegaRadiansPerSecond, false);
    }
    public void driveFieldRelative(ChassisSpeeds c) {
      drive(c.vxMetersPerSecond, c.vyMetersPerSecond, 0, true);
    }
    // public void driveDirect(ChassisSpeeds chassisSpeedsIn) {
    //   var speeds = ChassisSpeeds.discretize(chassisSpeedsIn, LoggedRobot.defaultPeriodSecs);
    //   var swerveModuleStates =
    //     m_kinematics.toSwerveModuleStates(speeds);
    //     SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    //     m_backLeft_0.setStateDirectly(swerveModuleStates[0]);
    //     m_backRight_1.setStateDirectly(swerveModuleStates[1]);
    //     m_frontRight_2.setStateDirectly(swerveModuleStates[2]);
    //     m_frontLeft_3.setStateDirectly(swerveModuleStates[3]);
    // }
//    @AutoLogOutput 
//    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    Logger.recordOutput(getName(), 1);
    SmartDashboard.putNumber("[Drivetrain]Gyro", m_gyro.getAngle());
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(-1*xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    SmartDashboard.putString("[Drivetrain]gyro", m_gyro.getRotation2d().toString());
    SmartDashboard.putString("[Drivetrain]module 0", swerveModuleStates[0].toString());
    SmartDashboard.putString("[Drivetrain]module 1", swerveModuleStates[1].toString());
    SmartDashboard.putString("[Drivetrain]module 2", swerveModuleStates[2].toString());
    SmartDashboard.putString("[Drivetrain]module 3", swerveModuleStates[3].toString());

    setModuleStates(swerveModuleStates);

    Logger.recordOutput("SwerveStates", swerveModuleStates);

    //if(RobotState.isTest()) {


   // }
    xSpeed_cur = xSpeed;
    ySpeed_cur = ySpeed;
    rot_cur = rot;
  }

  private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    m_backLeft_0.setDesiredState(swerveModuleStates[0]);
    m_backRight_1.setDesiredState(swerveModuleStates[1]);
    m_frontRight_2.setDesiredState(swerveModuleStates[2]);
    m_frontLeft_3.setDesiredState(swerveModuleStates[3]);
  }

  public void stop() {
    m_backLeft_0.stop();
    m_backRight_1.stop();
    m_frontRight_2.stop();
    m_frontLeft_3.stop();
    xSpeed_cur = 0;
    ySpeed_cur = 0;
    rot_cur = 0;
  }

  public ChassisSpeeds getSpeed() {
    return new ChassisSpeeds(xSpeed_cur, ySpeed_cur, rot_cur);
  }

  private ChassisSpeeds getRobotRelativeSpeeds(){
    var c = m_kinematics.toChassisSpeeds(getModuleStates());
    SmartDashboard.putString("[Drivetrain]Robot relative speeds", c.toString());
    return c;
  }

  private SwerveModuleState[] getModuleStates(){
      return new SwerveModuleState[]{
          m_backLeft_0.getState(),
          m_backRight_1.getState(),
          m_frontRight_2.getState(),
          m_frontLeft_3.getState()
      };
  }

  /** Updates the field relative position of the robot. */
  // public void updateOdometry() {
  //   m_odometry.update(
  //       m_gyro.getRotation2d(),
  //       getModulePositions());
  // }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_backLeft_0.getPosition(),
          m_backRight_1.getPosition(),
          m_frontRight_2.getPosition(),
          m_frontLeft_3.getPosition()
    };
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  } 

  public void resetPose(Pose2d aPose2d) {
//    m_odometry.resetPosition(m_gyro.getRotation2d(),
//         getModulePositions(), aPose2d );
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
          4.000,
          3.000,
          Units.degreesToRadians(540.000),
          Units.degreesToRadians(720.000)
        );
    }
    
    // public boolean DriveMeters(double meters) {
    //   if(first) {
    //     first = false;
    //     encoderDoubles[0] = m_frontLeft.getEncoderValue();
    //     encoderDoubles[1] = m_frontRight.getEncoderValue();
    //     encoderDoubles[2] = m_backLeft.getEncoderValue();
    //     encoderDoubles[3] = m_backRight.getEncoderValue();
    //     startencoder = Math.abs(java.util.Arrays.stream(encoderDoubles).mapToDouble(Double::doubleValue).average().orElse(0.0));
    //   }
    //   curencoder = Math.abs(java.util.Arrays.stream(curencoderDoubles).mapToDouble(Double::doubleValue).average().orElse(0.0));
    //   target = Math.abs((startencoder + rotationsToInch.calculateRotationsM(meters, 6.75, 6)) - java.util.Arrays.stream(curencoderDoubles).mapToDouble(Double::doubleValue).average().orElse(0.0));
    //   double remainingDistance = target - curencoder;
    //   if (Math.abs(Math.round(remainingDistance)) <= 0) {
    //     stop();
    //     return true;
    //   } else {
    //     drive(0, 0.1, 0, false);
    //     SmartDashboard.putNumber("[Drivetrain]Curencoder", curencoder);
    //     SmartDashboard.putNumber("[Drivetrain]encoder", startencoder);
    //     SmartDashboard.putNumber("[Drivetrain]target", target);
    //     return false;
    //   }
    // }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("[Drivetrain]raw X", m_gyro.getRawAccelX());
      SmartDashboard.putNumber("[Drivetrain]raw Y", m_gyro.getRawAccelY());

      SmartDashboard.putNumber("[Drivetrain]X accel", m_gyro.getWorldLinearAccelX());
      SmartDashboard.putNumber("[Drivetrain]Y accel", m_gyro.getWorldLinearAccelY());
      
    SmartDashboard.putNumber("[Drivetrain]battery voltage", RobotController.getBatteryVoltage());

    // This method will be called once per scheduler run
      poseEstimator.update(m_gyro.getRotation2d(), getModulePositions());
      field.setRobotPose(poseEstimator.getEstimatedPosition());
    
      SmartDashboard.putData("[Drivetrain]robotpose", field);

      SmartDashboard.putNumber("[Drivetrain]Gyro yaw", m_gyro.getYaw());
      SmartDashboard.putNumber("[Drivetrain]Gyro pitch", m_gyro.getPitch());
      SmartDashboard.putNumber("[Drivetrain]Gyro roll", m_gyro.getRoll());
      SmartDashboard.putNumber("[Drivetrain]Gyro angle", m_gyro.getAngle());

      // curencoderDoubles[0] = m_frontLeft.getEncoderValue();
      // curencoderDoubles[1] = m_frontRight.getEncoderValue();
      // curencoderDoubles[2] = m_backLeft.getEncoderValue();
      // curencoderDoubles[3] = m_backRight.getEncoderValue();
      
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
