// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.DriveConstants;
import java.lang.Math;

public class DriveTrainSubsystem extends SubsystemBase {
	/** Creates a new DriveTrain. */
	private WPI_TalonFX frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
	MecanumDrive drive;
	private final AHRS m_navx;

	int FL = 3; // Front Left
	int RL = 7; // Rear Left
	int FR = 0; // Front Right
	int RR = 4; // Rear Right

	// The motors on the left side of the drive.
	private final SpeedControllerGroup m_leftMotors =
	new SpeedControllerGroup(new WPI_TalonFX(FL),
							 new WPI_TalonFX(RL));

// The motors on the right side of the drive.
	private final SpeedControllerGroup m_rightMotors =
	new SpeedControllerGroup(new WPI_TalonFX(FR),
							 new WPI_TalonFX(RR));
	
	private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
	private DifferentialDriveOdometry m_odometry;

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
		m_navx = new AHRS();
		m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
		reverseEncoders();
		resetOdometry(new Pose2d());
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
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
	}
	public void zeroDriveTrainEncoders() {
		frontLeftMotor.setSelectedSensorPosition(0);
		frontRightMotor.setSelectedSensorPosition(0);
	}
	public void resetOdometry(Pose2d pose) {
		zeroDriveTrainEncoders();
		m_navx.reset();
		m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
	}
	public void setNeutralMode(NeutralMode neutralMode) {
		frontLeftMotor.setNeutralMode(neutralMode);
		rearLeftMotor.setNeutralMode(neutralMode);
		frontRightMotor.setNeutralMode(neutralMode);
		rearRightMotor.setNeutralMode(neutralMode);
	}
	public void arcadeDrive(double fwd, double rot) {
		m_drive.arcadeDrive(fwd, rot);
	  }
	  public double getHeading() {
		return m_navx.getRotation2d().getDegrees();
	  }
	  public void zeroHeading() {
		m_navx.reset();
	}
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		m_leftMotors.setVoltage(leftVolts);
		m_rightMotors.setVoltage(-rightVolts);
		m_drive.feed();
	}
	public double getLeftEncoderRate() {
		return frontLeftMotor.getSelectedSensorVelocity(0) * DriveConstants.ENCODER_CONSTANT * 10
				*DriveConstants.MAG;
	}
	public double getRightEncoderRate() {
		return -frontRightMotor.getSelectedSensorVelocity(0) * DriveConstants.ENCODER_CONSTANT * 10
				* DriveConstants.MAG;
	} 
	public float getYDisplacement() {
		return m_navx.getDisplacementY();
	}
	public float getYVelocity(){
		return m_navx.getVelocityY();
	}
	public float getXDisplacement() {
		return m_navx.getDisplacementX();
	}
	public float getXVelocity(){
		return m_navx.getVelocityY();
	}
	
	public void feedMotorSafety() {
		m_drive.feed();
	}
	public void reverseEncoders() {
		DriveConstants.MAG *= -1;
		resetOdometry(new Pose2d());
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
