// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

// import com.revrobotics.spark.SparkSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends Command {

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  // private final SparkSim driveMotorSim;

  private final RelativeEncoder m_driveEncoder;
  private final AnalogInput m_turningEncoder;

  private double ahhhhhhhhhhh = 0.0;

  private double encoderOffset = 0;

  private int moduleNumber = 0;
  @SuppressWarnings("unused")
  private final RelativeEncoder m_turningEncoderREV;

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      42, // 16.7 -- updated to 16.5
      0,
      0,
      new TrapezoidProfile.Constraints(
          Constants.SwerveConstants.kModuleMaxAngularVelocity,
          Constants.SwerveConstants.kModuleMaxAngularAcceleration));

  // // Gains are for example purposes only - must be determined for your own
  // robot!
  // private final SimpleMotorFeedforward m_driveFeedforward = new
  // SimpleMotorFeedforward(1, 1.5); // this was 3, changed to 1.5 because it was
  // driving too far
  // private final SimpleMotorFeedforward m_turnFeedforward = new
  // SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor and turning
   * encoder.
   *
   * @param driveMotorChannel      PWM output for the drive motor.
   * @param turningMotorChannel    PWM output for the turning motor.
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   */
  public SwerveModule(SwerveModuleConstants moduleConstants) {
    SmartDashboard.putNumber("[Swerve]tuneing", ahhhhhhhhhhh);
    m_driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    var driveConfig = new SparkMaxConfig();
    driveConfig.inverted(true);
    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // driveMotorSim = new SparkSim(m_driveMotor, DCMotor.getNEO(1));
    m_turningMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    var turningConfig = new SparkMaxConfig();
    turningConfig.inverted(true);
    m_turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = new AnalogInput(moduleConstants.cancoderID);

    m_turningEncoderREV = m_turningMotor.getEncoder();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    /*
     * m_driveEncoder.setPositionConversionFactor(driveAfterEncoderReduction);
     * m_driveEncoder.setVelocityConversionFactor(driveAfterEncoderReduction /
     * 60.0);
     * 
     * m_turningEncoderREV.setPositionConversionFactor(turnAfterEncoderReduction);
     */
    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerRotation(-2 * Math.PI);

    moduleNumber = moduleConstants.cancoderID;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // load the encoder offset
    // encoderOffset = Preferences.getDouble("encoder" + moduleNumber,
    // encoderOffset);
    // hard coding the offset because its better?
    switch (moduleNumber) {
      case 0:
        encoderOffset = -1.020057144401588;
        break;
      case 1:
        encoderOffset = -1.450295970644589;// 0.922783865280067
        break;
      case 2:
        encoderOffset = -1.903437227196342;// -13.00*Math.PI/180.00;
        break;
      case 3:
        encoderOffset = -4.41164894046481;
        break;
    }
    configAngleMotor();
    configDriveMotor();
  }

  // public SparkSim getDriveMotorSim() {
  // return driveMotorSim;
  // }

  private double encoderValue() {
    var retVal = getRawAngle();
    // SmartDashboard.putNumber("[Swerve]module " + moduleNumber, retVal);
    if (RobotState.isTest()) {
      SmartDashboard.putNumber("[Swerve]encoder raw " + moduleNumber, retVal);
    }

    SmartDashboard.putNumber("[Swerve]encoder " + moduleNumber, (retVal * 1000) / 1000.0);
    SmartDashboard.putNumber("[Swerve]encoder degrees " + moduleNumber, (retVal * (180 / Math.PI) * 1000) / 1000.0);

    retVal = (retVal + encoderOffset) % (2.0 * Math.PI); // apply offset for this encoder and map it back onto [0, 2pi]
    // might need this so we're in the same range as the pid controller is
    // expecting.
    // retVal = retVal - Math.PI;
    if (RobotState.isTest()) {
      SmartDashboard.putNumber("[Swerve]encoder adjusted " + moduleNumber, retVal);
    }

    SmartDashboard.putNumber("[Swerve]EncoderValue() " + moduleNumber, retVal);
    return (retVal);
  }

  private double getRawAngle() {
    var retVal = m_turningEncoder.getVoltage() / RobotController.getVoltage5V(); // convert voltage to %
    retVal = 2.0 * Math.PI * retVal; // get % of circle encoder is reading
    return retVal;
  }

  public void resetEncoder() {
    m_driveMotor.getEncoder().setPosition(0.0);

    // get the turning encoder and write it to preferences
    encoderOffset = m_turningEncoder.getVoltage() / RobotController.getVoltage5V(); // convert voltage to %
    encoderOffset = 2.0 * Math.PI * encoderOffset; // get % of circle encoder is reading
    encoderOffset = (2.0 * Math.PI) - encoderOffset;
    // Preferences.initDouble("encoder" + moduleNumber, encoderOffset);
    Preferences.setDouble("encoder" + moduleNumber, encoderOffset);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    var s = getConvertedVelocity();
    return new SwerveModuleState(
        s, new Rotation2d(encoderValue()));
  }

  private double getConvertedVelocity() {
    return (m_driveEncoder.getVelocity() / (60.0 * SwerveConstants.gearboxRatio))
        * (SwerveConstants.wheeldiameter * Math.PI);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // encode is % rotations
    var retVal = 1
        * ((m_driveEncoder.getPosition() / SwerveConstants.gearboxRatio) * SwerveConstants.wheeldiameter * Math.PI); // distance
                                                                                                                     // in
                                                                                                                     // whatever
                                                                                                                     // units
                                                                                                                     // the
                                                                                                                     // wheel
                                                                                                                     // diameter
                                                                                                                     // is
    // KB ^^^^ This is from 1 meter testing dont move/change
    return new SwerveModulePosition(retVal, new Rotation2d(encoderValue()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SmartDashboard.putNumber("[Swerve]Pre Optimize angle target degrees " + moduleNumber,
        desiredState.angle.getDegrees());
    // Optimize the reference state to avoid spinning further than 90 degrees
    SmartDashboard.putNumber("[Swerve]turn encoder" + moduleNumber, encoderValue());
    @SuppressWarnings("deprecation")
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(encoderValue()));
    SmartDashboard.putNumber("[Swerve]After Optimize angle target degrees " + moduleNumber, state.angle.getDegrees());
    // Calculate the drive output from the drive PID controller.

    /*
     * this bit from the example uses driveencoder.getrate() - Get the current rate
     * of the
     * encoder. Units are distance per second as scaled by the value from
     * setDistancePerPulse().
     * rev encoder doesn't have this value only
     * .getVelocity - Get the velocity of the motor. This returns the native units
     * of 'rotations per second'
     * by default, and can be changed by a scale factor using
     * setVelocityConversionFactor().
     */

    // final double driveOutput =
    // m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
    // state.speedMetersPerSecond);

    // final double driveFeedforward =
    // m_driveFeedforward.calculate(state.speedMetersPerSecond);
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(encoderValue(), state.angle.getRadians());
    // SmartDashboard.putNumber("[Swerve]pid " + moduleNumber, turnOutput);

    SmartDashboard.putNumber("[Swerve]Setpoint velocity", m_turningPIDController.getSetpoint().velocity);
    // final double turnFeedforward =
    // m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    // SmartDashboard.putNumber("[Swerve]turnFeedforward",turnFeedforward);
    if (RobotState.isAutonomous()) {
      // m_driveMotor.set(driveOutput / SwerveConstants.MaxMetersPersecond);
      m_driveMotor.set(state.speedMetersPerSecond / (1 * SwerveConstants.MaxMetersPersecond));
      // System.out.println("Output: " + driveOutput + " Feedforward: " +
      // driveFeedforward);
      m_turningMotor.set(turnOutput / SwerveConstants.kModuleMaxAngularVelocity);
    } else if (RobotState.isTeleop()) {
      m_driveMotor.set(state.speedMetersPerSecond);
      m_turningMotor.set(turnOutput / SwerveConstants.kModuleMaxAngularVelocity);
    }

    SmartDashboard.putNumber("[Swerve]m_driveMotor set " + moduleNumber,
        state.speedMetersPerSecond / SwerveConstants.MaxMetersPersecond);
    SmartDashboard.putNumber("[Swerve]m_turningMotor set " + moduleNumber,
        turnOutput / SwerveConstants.kModuleMaxAngularVelocity);

    SmartDashboard.putNumber("[Swerve]m_driveMotor actual" + moduleNumber, getConvertedVelocity());
    SmartDashboard.putNumber("[Swerve]m_turningMotor actual" + moduleNumber, m_turningMotor.get());

    SmartDashboard.putNumber("[Swerve]drive encoder" + moduleNumber, m_driveMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("[Swerve]turn encoder" + moduleNumber, encoderValue());

    if (RobotState.isTest()) {
      SmartDashboard.putNumber("[Swerve]turnOutput", turnOutput);
      // SmartDashboard.putNumber("[Swerve]Drive", ((driveOutput + driveFeedforward)
      // /2.1) /2);
      // SmartDashboard.putNumber("[Swerve]Turning stuff", Math.max(turnOutput,
      // turnFeedforward));
      // SmartDashboard.putNumber("[Swerve]Turning stuff", turnOutput +
      // turnFeedforward);
      SmartDashboard.putNumber("[Swerve]target " + moduleNumber, state.angle.getRadians());
    }

  }

  public void stop() {
    m_driveMotor.stopMotor();
    m_turningMotor.stopMotor();
  }

  // public void setStateDirectly(SwerveModuleState desiredState) {
  // // Optimize the reference state to avoid spinning further than 90 degrees
  // desiredState.angle =
  // desiredState.angle.minus(Rotation2d.fromRadians(encoderOffset));
  // @SuppressWarnings("deprecation")
  // SwerveModuleState state =
  // SwerveModuleState.optimize(desiredState, new Rotation2d(getRawAngle()));
  // m_driveMotor.set(state.speedMetersPerSecond);
  // m_turningMotor.setVoltage(toPositiveAngle(state.angle.getRadians()) *
  // RobotController.getVoltage5V() / (2.0 * Math.PI));
  // //m_driveMotor.getClosedLoopController().setReference(state.speedMetersPerSecond,
  // SparkMax.ControlType.kVelocity);
  // //m_turningMotor.getClosedLoopController().setReference(state.angle.getRadians(),
  // SparkMax.ControlType.kPosition);
  // SmartDashboard.putBoolean("[Swerve]Driving auto", true);
  // }

  // private double toPositiveAngle(double radians) {
  // return radians < 0 ? (radians + 2.0 * Math.PI) : radians;
  // }

  private void configAngleMotor() {
    var turnConfig = new SparkMaxConfig();
    turnConfig.smartCurrentLimit(40, 40);
    turnConfig.inverted(true);
    turnConfig.closedLoop.p(16.5);
    turnConfig.closedLoop.outputRange(-Math.PI, Math.PI);
    m_turningMotor.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configDriveMotor() {
    var driveConfig = new SparkMaxConfig();
    driveConfig.smartCurrentLimit(40, 40);
    driveConfig.inverted(true);
    driveConfig.encoder.positionConversionFactor(1); // Constants.SwerveConstants.driveConversionFactor);
    driveConfig.closedLoop.p(1);
    m_driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_driveEncoder.setPosition(0.0);
  }

  public double getEncoderValue() {
    return m_driveMotor.getEncoder().getPosition();
  }

}
