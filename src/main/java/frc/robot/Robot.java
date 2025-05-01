// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
// import org.springframework.beans.factory.annotation.Autowired;
// import org.springframework.stereotype.Component;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


// @Component
public class Robot extends LoggedRobot {
  
    public static final boolean IS_COMPETITION = false;
  private Command m_autonomousCommand;

  private Timer m_gcTimer = new Timer();
  private Timer timer = new Timer();
  // @Autowired
  private RobotContainer m_robotContainer;
  private boolean firstInit = true;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    SmartDashboard.putNumber("WheelPointAngle", 1);
    UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(1280, 800);
              camera.setWhiteBalanceAuto();
              camera.setFPS(30);
    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
    Logger.start();
    Logger.disableConsoleCapture();
  }
  @Override
  public void robotInit () {
        // Set up data receivers & replay source
        
    // Initialize URCL
    
    // Start AdvantageKit logger
    
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

    m_gcTimer.start();
    timer = new Timer();
    // No longer needed since we use Spring to wire components
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {


    Logger.recordOutput("Auto number", auto);
    if (m_gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
    this.m_robotContainer.periodic();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    if (firstInit) {
      Command AutonStartCommand =
                    FollowPathCommand.warmupCommand().andThen(PathfindingCommand.warmupCommand());
            AutonStartCommand.schedule();
      firstInit = false;
    }
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @AutoLogOutput
  int auto = 0;
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    this.m_robotContainer.autonomousInit();
    auto++;
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      Commands.waitSeconds(0.01).andThen(m_autonomousCommand).schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    this.m_robotContainer.autonomousPeriodic();
  }

  @Override
  public void teleopInit() {
    timer.reset();
    timer.start();
    ii = false;
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    this.m_robotContainer.teleopInit();
  }
  boolean ii = false;
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    this.m_robotContainer.teleopPeriodic();
    if(timer.advanceIfElapsed(155) || ii) {
      System.out.println("CHECK THE SWERVE");
      ii = true;
      //CommandScheduler.getInstance().cancel(new TelopSwerve(null, null, null, null, null));
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
  }
}