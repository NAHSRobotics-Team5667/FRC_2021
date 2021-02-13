// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import java.lang.Math;

public class DriveTrainSubsystem extends SubsystemBase {
	/** Creates a new DriveTrain. */
	private WPI_TalonFX frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
	MecanumDrive drive;
	private DifferentialDrive m_drive;
	int FL = 3; // Front Left
	int RL = 7; // Rear Left
	int FR = 0; // Front Right
	int RR = 4; // Rear Right
	
	public DriveTrainSubsystem() {
		frontLeftMotor = new WPI_TalonFX(FL);
		rearLeftMotor = new WPI_TalonFX(RL);
		frontRightMotor = new WPI_TalonFX(FR);
		rearRightMotor = new WPI_TalonFX(RR);
		drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		frontLeftMotor.configFactoryDefault();
		rearLeftMotor.configFactoryDefault();
		frontRightMotor.configFactoryDefault();
		rearRightMotor.configFactoryDefault();
		TalonFXConfiguration falconConfig = new TalonFXConfiguration();
		// falconConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		// falconConfig.openloopRamp = .8;

		// frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		// frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

		setNeutralMode(NeutralMode.Brake);

		frontRightMotor.configAllSettings(falconConfig);
		frontLeftMotor.configAllSettings(falconConfig);

		rearRightMotor.configAllSettings(falconConfig);
		rearLeftMotor.configAllSettings(falconConfig);

		rearLeftMotor.follow(frontLeftMotor);

		rearRightMotor.follow(frontRightMotor);

	//	m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

	}

	/**
	 * Drive the robot using cartesian coordinates from a joystick
	 * 
	 * @param xSpeed    - The x axis (Left/Right joystick)
	 * @param ySpeed    - The y axis (Forward/Backward joystick)
	 * @param zRotation - The z axis (Rotation)
	 */
	
	public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {

		this.drive.driveCartesian(0.7*xSpeed*xSpeed * Math.signum(xSpeed),0.7*ySpeed*ySpeed * Math.signum(ySpeed), 0.5*zRotation);
		//this.drive.driveCartesian(0,0.75, 0);

	}
	public void setNeutralMode(NeutralMode neutralMode) {
		frontLeftMotor.setNeutralMode(neutralMode);
		rearLeftMotor.setNeutralMode(neutralMode);
		frontRightMotor.setNeutralMode(neutralMode);
		rearRightMotor.setNeutralMode(neutralMode);
	}
	
	/**
	 * Stop the robot from driving
	 */
	public void stop() {

		this.drive.stopMotor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
