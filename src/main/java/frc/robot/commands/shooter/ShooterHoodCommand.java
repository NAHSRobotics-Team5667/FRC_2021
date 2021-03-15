// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterHoodCommand extends CommandBase {
  private double initialDegrees;
  private double endDegrees;
  private ShooterSubsystem m_shooter;

  /**
   * Creates a ShooterHoodCommand object.
   * 
   * @param initialDegrees initial degrees of the shooter hood.
   * @param endDegrees     end degrees of the shooter hood.
   * @param m_shooter      shooter subsystem.
   */
  public ShooterHoodCommand(double initialDegrees, double endDegrees, ShooterSubsystem m_shooter) {
    this.initialDegrees = initialDegrees;
    this.endDegrees = endDegrees;
    this.m_shooter = m_shooter;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.stopHood();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setHood(m_shooter.degreesToHoodTicks(endDegrees, initialDegrees), 0.01);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.getHoodAngle() - endDegrees <= Math.abs(1); // 1 degree tolerance
  }
}
