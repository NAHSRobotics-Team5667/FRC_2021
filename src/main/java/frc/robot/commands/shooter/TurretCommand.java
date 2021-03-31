// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
	private double startAngle;
	private double endAngle;
	private TurretSubsystem m_turret;

	private PIDController m_controller = new PIDController(Constants.ShooterConstants.TURRET_kP,
			Constants.ShooterConstants.TURRET_kI, Constants.ShooterConstants.TURRET_kD);

	/**
	 * Creates a TurretCommand object.
	 * 
	 * @param startAngle initial degrees of the shooter hood.
	 * @param endAngle   end degrees of the shooter hood.
	 * @param m_shooter  shooter subsystem.
	 */
	public TurretCommand(TurretSubsystem m_turret) {
		this.m_turret = m_turret;
		addRequirements(m_turret);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_turret.stopTurret();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(RobotContainer.getController().getXButton()){
			m_turret.startTurret(Constants.ShooterConstants.TURRET_SPEED, true);
		}
		else if(RobotContainer.getController().getBButton()){
			m_turret.startTurret(Constants.ShooterConstants.TURRET_SPEED, false);
		}
		// use PID Controller to adjust turret angle
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_turret.stopTurret();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
		//return Math.abs(m_turret.getTurretAngle() - endAngle) <= 1; // ends when turret is in direction of target, with
																	// 1 degree of inaccuracy
	}
}
