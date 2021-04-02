// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
	private WPI_TalonFX m_turret;
	private double turretAngle;
	private double initialTurretAngle;

	/**
	 * Creates a new TurretSubsystem.
	 * 
	 * @param turretAngle - The initial angle of the turret.
	 */
	public TurretSubsystem(double turretAngle) {
		m_turret = new WPI_TalonFX(ShooterConstants.TURRET_ID);
		m_turret.setNeutralMode(NeutralMode.Brake);
		m_turret.setSelectedSensorPosition(0);

		// m_turret.setInverted(true); // positive is clockwise

		this.turretAngle = turretAngle;
		this.initialTurretAngle = turretAngle;
	}

	private void updateTurretAngle(double turretTicks) {
		turretAngle = ((turretTicks * 360) / (Constants.FALCON_CPR * (1 / Constants.ShooterConstants.TURRET_GEAR_RATIO))) + initialTurretAngle;
	}

	// Stops turret motor from moving
	public void stopTurret() {
		m_turret.stopMotor();
	}

	/**
	 * Starts the turret in a given direction.
	 * 
	 * @param dir direction of the turret. (true is clockwise, false is counter-clockwise)
	 */
	public void startTurret(double speed) {
		System.out.println(getTurretAngle());
		if (getTurretAngle() < -85) {
			m_turret.set(Constants.ShooterConstants.TURRET_SPEED); //check, might have to reverse
		} else if (getTurretAngle() > 85) {
			m_turret.set(-Constants.ShooterConstants.TURRET_SPEED);
		} else {
			m_turret.set(speed);
		}
	}
	
	public void search(double speed) {
		//will finish, committing for now.
	}

	/**
	 * @return Angle of the turret in relation to the robot.
	 */
	public double getTurretAngle() {
		return turretAngle;
	}

	@Override
	public void periodic() {
		updateTurretAngle(m_turret.getSelectedSensorPosition());
	}
}