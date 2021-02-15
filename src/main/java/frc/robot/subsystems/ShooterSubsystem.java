// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ShooterSubsystem extends SubsystemBase {
  // hood
  private WPI_TalonFX m_hood;
  private double hoodAngle;

  /**
   * @param hoodAngle The initial angle of the hood.
   */
  public ShooterSubsystem(double hoodAngle) {
    m_hood = new WPI_TalonFX(Constants.ShooterConstants.HOOD_ID);
    this.hoodAngle = hoodAngle;
  }

  public void stopHood() {
    m_hood.stopMotor();
  }

  /**
   * @return Angle of the powercell in relation to the ground when it exits the shooter.
   */
  public double getHoodAngle() {
    return hoodAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
