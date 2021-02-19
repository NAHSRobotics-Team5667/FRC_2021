// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ShooterSubsystem extends SubsystemBase {
  // hood
  private WPI_TalonFX m_hood;
  private double initialHoodAngle;
  private double hoodAngle;

  /**
   * @param hoodAngle The initial angle of the hood.
   */
  public ShooterSubsystem(double hoodAngle) {
    m_hood = new WPI_TalonFX(Constants.ShooterConstants.HOOD_ID);
    m_hood.setInverted(false); // positive makes the angle larger, negative makes the angle smaller
    m_hood.setNeutralMode(NeutralMode.Brake);
    m_hood.setVoltage(12);

    this.initialHoodAngle = hoodAngle;
    this.hoodAngle = hoodAngle;
  }

  public void stopHood() {
    m_hood.stopMotor();
  }

  /**
   * @param tickDifference difference in ticks for the hood motor. Can be obtained by calling the degreesToTicks() function.
   * @param percentOutput percentage of voltage to output to the motor.
   */
  public void setHood(double tickDifference, double percentOutput) {
    double targetTicks = m_hood.getSelectedSensorPosition() + tickDifference;

    if (m_hood.getSelectedSensorPosition() < targetTicks) m_hood.set(ControlMode.PercentOutput, percentOutput);
    else if (m_hood.getSelectedSensorPosition() > targetTicks) m_hood.set(ControlMode.PercentOutput, -percentOutput);
  }

  /**
   * @param distance normal distance from inner power port
   */
  public double distanceToDegrees(double distance) {
    return -1; // placeholder, requires data from testing
  }

  /**
   * @param endDegrees end degree position of the hood.
   * @param initialDegrees initial degree position of the hood.
   * @return Tick difference for the Falcon 500.
   */
  public double degreesToTicks(double endDegrees, double initialDegrees) {
    return ((Constants.FALCON_CPR * (1 / Constants.ShooterConstants.HOOD_GEAR_RATIO)) * (endDegrees - initialDegrees)) / 360;
  }

  /**
   * @param hoodTicks encoder count from Falcon 500.
   */
  private void updateHoodAngle(double hoodTicks) {
    hoodAngle = ((hoodTicks * 360) / (Constants.FALCON_CPR * (1 / Constants.ShooterConstants.HOOD_GEAR_RATIO))) + initialHoodAngle;
  }

  /**
   * @return Angle of the powercell in relation to the ground when it exits the shooter.
   */
  public double getHoodAngle() {
    return hoodAngle;
  }

  @Override
  public void periodic() {
    updateHoodAngle(m_hood.getSelectedSensorPosition());
  }
}
