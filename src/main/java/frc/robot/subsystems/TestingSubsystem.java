// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestingSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SparkMax test;
  public TestingSubsystem(int CADID) {
    test = new SparkMax(90, MotorType.kBrushless);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void stuff(double joy) {
    test.set(joy);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean resetEncoder() {
    test.getEncoder().setPosition(0);
    return true;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("endcoder", test.getEncoder().getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
