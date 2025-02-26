// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.TimedRobot;
// import org.springframework.beans.factory.annotation.Autowired;
// import org.springframework.stereotype.Component;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// @Component
public class Robot extends TimedRobot {
  private static final Constants.RobotMode JAVA_SIM_MODE = Constants.RobotMode.SIM;
    public static final Constants.RobotMode CURRENT_ROBOT_MODE = isReal() ? Constants.RobotMode.REAL : JAVA_SIM_MODE;
    public static final boolean IS_COMPETITION = true;
  private Command m_autonomousCommand;

  private Timer m_gcTimer = new Timer();
  private Timer timer = new Timer();
  // @Autowired
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit () {
        // Set up data receivers & replay source
        

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        FollowPathCommand.warmupCommand().schedule();

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
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    this.m_robotContainer.autonomousInit();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
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