// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class TurretSubsystem extends SubsystemBase {

	private WPI_TalonFX m_turret;
	private double turretAngle;

	/**
	 * Creates a new TurretSubsystem.
	 * 
	 * @param m_turret    - The turret motor.
	 * @param turretAngle - The initial angle of the turret.
	 */
	public TurretSubsystem() {
		m_turret = new WPI_TalonFX(ShooterConstants.TURRET_ID);

	}

	// Stops turret motor from moving
	public void stopTurret() {
		m_turret.stopMotor();
	}
	//Starts turret motor
	public void startTurret(double speed, boolean dir){
		if(dir){
			m_turret.set(speed);
		}
		else{
			m_turret.set(-speed);
		}
		}
	/**
	 * @return Angle of the turret in relation to the robot.
	 */
	public double getTurretAngle() {
		return turretAngle;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
