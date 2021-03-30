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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import frc.robot.Constants.DriveConstants;
import java.lang.Math;

public class DriveTrainSubsystem extends SubsystemBase {
	/** Creates a new DriveTrain. */
	private WPI_TalonFX frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
	MecanumDrive drive;
	private final AHRS m_navx;


	// The motors on the left side of the drive.
	private final SpeedControllerGroup m_leftMotors;
// The motors on the right side of the drive.
	private final SpeedControllerGroup m_rightMotors;
	private DriveModes m_driveMode = DriveModes.MANUAL;

	private ShuffleboardTab graphTab = Shuffleboard.getTab("Graphs");
	private ShuffleboardTab compTab = Shuffleboard.getTab("Teleop");

	private NetworkTable live_dashboard = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
	private NetworkTableEntry robotX = live_dashboard.getEntry("robotX");
	private NetworkTableEntry robotY = live_dashboard.getEntry("robotY");
	private NetworkTableEntry robotHeading = live_dashboard.getEntry("robotHeading");

	private MecanumDriveOdometry m_odometry;
	public static enum DriveModes {
		MANUAL(0), AUTO(1);

		private int mode;

		private DriveModes(int mode) {
			this.mode = mode;
		}

		public int getMode() {
			return mode;
		}
	}

	public DriveTrainSubsystem(AHRS gyro, WPI_TalonFX frontRight, WPI_TalonFX frontLeft, WPI_TalonFX rearRight, WPI_TalonFX rearLeft) {
		frontLeftMotor = frontLeft;
		rearLeftMotor = rearLeft;
		frontRightMotor = frontRight;
		rearRightMotor = rearRight;
		m_navx = gyro;

		m_leftMotors = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
		m_rightMotors = new SpeedControllerGroup(frontRightMotor, rearRightMotor);
		drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		frontLeftMotor.configFactoryDefault();
		rearLeftMotor.configFactoryDefault();
		frontRightMotor.configFactoryDefault();
		rearRightMotor.configFactoryDefault();
		TalonFXConfiguration falconConfig = new TalonFXConfiguration();
		m_odometry = new MecanumDriveOdometry(Constants.DriveConstants.kDriveKinematics,new Rotation2d(getHeading()) , new Pose2d(Constants.DriveConstants.startY, Constants.DriveConstants.startX, new Rotation2d()));
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
		
		//rearLeftMotor.follow(frontLeftMotor);
		//rearRightMotor.follow(frontRightMotor);

	}

	/**
	 * Drive the robot using cartesian coordinates from a joystick
	 * 
	 * @param xSpeed    - The x axis (Left/Right joystick)
	 * @param ySpeed    - The y axis (Forward/Backward joystick)
	 * @param zRotation - The z axis (Rotation)
	 */
	
	public void driveCartesian(double xSpeed, double ySpeed, double zRotation, boolean slowMode, boolean doubleSlowMode) {

		if (!slowMode && !doubleSlowMode) this.drive.driveCartesian(0.7*xSpeed*xSpeed * Math.signum(xSpeed),0.7*ySpeed*ySpeed * Math.signum(ySpeed), 0.5*zRotation, m_navx.getAngle());
		else if (slowMode && !doubleSlowMode) this.drive.driveCartesian(0.5*xSpeed*xSpeed * Math.signum(xSpeed),0.5*ySpeed*ySpeed * Math.signum(ySpeed), 0.3*zRotation, m_navx.getAngle());
		else if (slowMode && doubleSlowMode) this.drive.driveCartesian(0.28*xSpeed*xSpeed * Math.signum(xSpeed),0.28*ySpeed*ySpeed * Math.signum(ySpeed), (0.4 * 0.3)*zRotation, m_navx.getAngle());
		else if (!slowMode && doubleSlowMode) this.drive.driveCartesian(0.28*xSpeed*xSpeed * Math.signum(xSpeed),0.28*ySpeed*ySpeed * Math.signum(ySpeed), (0.4 * 0.3)*zRotation, m_navx.getAngle());
		//this.drive.driveCartesian(0,0.75, 0);

	}

	/**
	 * Get the current drive mode
	 * 
	 * @return - The current drive mode
	 */
	public DriveModes getDriveMode() {
		return m_driveMode;
	}

	/**
	 * Set the current drive mode
	 * 
	 * @param mode - The current drive train mode
	 */
	public void setDriveMode(DriveModes mode) {
		m_driveMode = mode;
	}

	public void driveVoltage(MecanumDriveMotorVoltages voltages){
		frontLeftMotor.set(voltages.frontLeftVoltage);
		frontRightMotor.set(voltages.frontRightVoltage);
		rearLeftMotor.set(voltages.rearLeftVoltage);
		rearRightMotor.set(voltages.rearRightVoltage);
		drive.feed();
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
	/**
	 * Controls the left and right sides of the drive directly with voltages.
	 *
	 * @param leftVolts  the commanded left output
	 * @param rightVolts the commanded right output
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		SmartDashboard.putNumber("raw_lv", leftVolts);
		SmartDashboard.putNumber("raw_rv", -rightVolts);

		m_leftMotors.set(leftVolts);
		m_rightMotors.set(-rightVolts);
		drive.feed();
	}

	/**
	 * Controls the left and right sides of the drive directly with voltages.
	 *
	 * @param leftVolts  the commanded left output
	 * @param rightVolts the commanded right output
	 */
	public void tankDriveVoltsReverse(double leftVolts, double rightVolts) {
		SmartDashboard.putNumber("raw_lv", leftVolts);
		SmartDashboard.putNumber("raw_rv", -rightVolts);

		m_leftMotors.set(-rightVolts);
		m_rightMotors.set(leftVolts);
		drive.feed();
	}
	public void setNeutralMode(NeutralMode neutralMode) {
		frontLeftMotor.setNeutralMode(neutralMode);
		rearLeftMotor.setNeutralMode(neutralMode);
		frontRightMotor.setNeutralMode(neutralMode);
		rearRightMotor.setNeutralMode(neutralMode);
	}
	// public void arcadeDrive(double fwd, double rot) {
	// 	m_drive.arcadeDrive(fwd, rot);
	//   }
	  public double getHeading() {
		return m_navx.getRotation2d().getDegrees();
	  }
 
	  public double getLeftEncoderPosition() {
		return frontLeftMotor.getSelectedSensorPosition(0) * DriveConstants.ENCODER_CONSTANT
				* DriveConstants.MAG;
	}
	public double getRightEncoderPosition() {
		return -frontRightMotor.getSelectedSensorPosition(0) * DriveConstants.ENCODER_CONSTANT
				* DriveConstants.MAG;
	}
	  public void zeroHeading() {
		m_navx.reset();
	}
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	
	public MecanumDriveWheelSpeeds getWheelSpeeds(){
		return new MecanumDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate(), getRearLeftEncoderRate(), getRearRightEncoderRate());
	}
	public double getLeftEncoderRate() {
		return frontLeftMotor.getSelectedSensorVelocity(0) * DriveConstants.ENCODER_CONSTANT * 10
				*DriveConstants.MAG;
	}
	public double getRearLeftEncoderRate() {
		return rearLeftMotor.getSelectedSensorVelocity(0) * DriveConstants.ENCODER_CONSTANT * 10
				*DriveConstants.MAG;
	}
	public double getRightEncoderRate() {
		return -frontRightMotor.getSelectedSensorVelocity(0) * DriveConstants.ENCODER_CONSTANT * 10
				* DriveConstants.MAG;
	} 
	public double getRearRightEncoderRate() {
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
		drive.feed();
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
		m_odometry.update(Rotation2d.fromDegrees(getHeading()), getWheelSpeeds());

		robotX.setDouble(Units.metersToFeet(m_odometry.getPoseMeters().getTranslation().getX()));
		robotY.setDouble(Units.metersToFeet(m_odometry.getPoseMeters().getTranslation().getY()));
		robotHeading.setDouble(Units.metersToFeet(m_odometry.getPoseMeters().getRotation().getRadians()));	}
}
