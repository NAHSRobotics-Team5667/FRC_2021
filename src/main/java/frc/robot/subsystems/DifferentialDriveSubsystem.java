/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DifferentialDriveSubsystem extends SubsystemBase {

	private WPI_TalonFX m_rightMaster, m_leftMaster, m_rightSlave, m_leftSlave;

	private DifferentialDrive m_drive;

	private final AHRS m_gyro;

	private DifferentialDriveOdometry m_odometry;

	private DriveModes m_driveMode = DriveModes.MANUAL;

	private ShuffleboardTab graphTab = Shuffleboard.getTab("Graphs");
	private ShuffleboardTab compTab = Shuffleboard.getTab("Teleop");

	private NetworkTable live_dashboard = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
	private NetworkTableEntry robotX = live_dashboard.getEntry("robotX");
	private NetworkTableEntry robotY = live_dashboard.getEntry("robotY");
	private NetworkTableEntry robotHeading = live_dashboard.getEntry("robotHeading");

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

	/**
	 * Create a Drive Train subsystem
	 * 
	 * @param rightMaster - The right falcon with encoder
	 * @param leftMaster  - The left falcon with encoder
	 * @param rightSlave  - The right falcon without encoder
	 * @param leftSlave   - The left falcon without encoder
	 * @param gyro        - The gyro
	 */
	public DifferentialDriveSubsystem(AHRS gyro, WPI_TalonFX rightMaster, WPI_TalonFX leftMaster, WPI_TalonFX rightSlave,
			WPI_TalonFX leftSlave) {

		m_rightMaster = rightMaster;
		m_leftMaster = leftMaster;
		m_rightSlave = rightSlave;
		m_leftSlave = leftSlave;

		m_rightMaster.configFactoryDefault();
		m_rightSlave.configFactoryDefault();

		m_leftMaster.configFactoryDefault();
		m_leftSlave.configFactoryDefault();

		TalonFXConfiguration falconConfig = new TalonFXConfiguration();
		falconConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		falconConfig.openloopRamp = .8;

		m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

		setNeutralMode(NeutralMode.Coast);

		m_rightMaster.configAllSettings(falconConfig);
		m_leftMaster.configAllSettings(falconConfig);

		m_leftSlave.follow(m_leftMaster);

		m_rightSlave.follow(m_rightMaster);

		m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

		m_gyro = gyro;

		m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
		reverseEncoders();
		resetOdometry(new Pose2d());
		// outputTelemetry();
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());

		robotX.setDouble(Units.metersToFeet(m_odometry.getPoseMeters().getTranslation().getX()));
		robotY.setDouble(Units.metersToFeet(m_odometry.getPoseMeters().getTranslation().getY()));
		robotHeading.setDouble(Units.metersToFeet(m_odometry.getPoseMeters().getRotation().getRadians()));
	}

	/**
	 * Sets the neutral mode for the drive train
	 * 
	 * @param neutralMode the desired neutral mode
	 */
	public void setNeutralMode(NeutralMode neutralMode) {
		m_leftMaster.setNeutralMode(neutralMode);
		m_leftSlave.setNeutralMode(neutralMode);
		m_rightMaster.setNeutralMode(neutralMode);
		m_rightSlave.setNeutralMode(neutralMode);
	}

	/**
	 * Drive the robot using arcade control
	 * 
	 * @param throttle - How fast it should drive (-1, 1)
	 * @param angle    - Change in heading
	 */
	public void drive(double throttle, double angle, boolean isQuickTurn) {
		m_drive.arcadeDrive(throttle, angle);
	}

	public void driveCheese(double throttle, double angle, boolean isQuickTurn) {
		m_drive.curvatureDrive(throttle, angle, isQuickTurn);
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

	/**
	 * Stop all motors
	 */
	public void stop() {
		m_drive.stopMotor();
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Returns the current wheel speeds of the robot.
	 *
	 * @return The current wheel speeds.
	 */
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose - The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		zeroDriveTrainEncoders();
		m_gyro.reset();
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

		m_leftMaster.set(leftVolts);
		m_rightMaster.set(-rightVolts);
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

		m_leftMaster.set(-rightVolts);
		m_rightMaster.set(leftVolts);
	}

	/**
	 * Resets the drive encoders to currently read a position of 0.
	 */
	public void zeroDriveTrainEncoders() {
		m_rightMaster.setSelectedSensorPosition(0);
		m_leftMaster.setSelectedSensorPosition(0);
	}

	/**
	 * Gets the average distance of the two encoders.
	 *
	 * @return the average of the two encoder readings
	 */
	public double getAverageEncoderDistance() {
		return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
	}

	/**
	 * Zeroes the heading of the robot.
	 */
	public void zeroHeading() {
		m_gyro.reset();
	}

	/**
	 * returns left encoder position
	 * 
	 * @return left encoder position
	 */
	public double getLeftEncoderPosition() {
		return m_leftMaster.getSelectedSensorPosition(0) * Constants.DriveConstants.ENCODER_CONSTANT
				* DriveConstants.MAG;
	}

	/**
	 * Returns right encoder position
	 * 
	 * @return right encoder position
	 */
	public double getRightEncoderPosition() {
		return -m_rightMaster.getSelectedSensorPosition(0) * DriveConstants.ENCODER_CONSTANT
				* Constants.DriveConstants.MAG;
	}

	/**
	 * Get the left encoder velocity
	 * 
	 * @return Get the left encoder velocity in m/s
	 */
	public double getLeftEncoderRate() {
		return m_leftMaster.getSelectedSensorVelocity(0) * DriveConstants.ENCODER_CONSTANT * 10
				* Constants.DriveConstants.MAG;
	}

	/**
	 * Geth the right encoder velocity
	 * 
	 * @return Get the right encoder velocity in m/s
	 */
	public double getRightEncoderRate() {
		return -m_rightMaster.getSelectedSensorVelocity(0) * Constants.DriveConstants.ENCODER_CONSTANT * 10
				* DriveConstants.MAG;
	}

	/**
	 * Returns the heading of the robot in form required for odometry.
	 *
	 * @return the robot's heading in degrees, from 180 to 180 with positive value
	 *         for left turn.
	 */
	public double getHeading() {
		return -m_gyro.getAngle();
	}

	/**
	 * Feed the motor safety during auto
	 */
	public void feedMotorSafety() {
		m_drive.feed();
	}

	public void reverseEncoders() {
		Constants.DriveConstants.MAG *= -1;
		resetOdometry(new Pose2d());
	}

	/**
	 * Output the Drive Train telemtry
	 */
	public void outputTelemetry() {
		compTab.addString("Pose", new Supplier<String>() {
			@Override
			public String get() {
				return m_odometry.getPoseMeters().toString();
			}
		});
	}

	/**
	 * Outputs for debugging
	 */
	public void debug() {

		graphTab.addNumber("r_pos", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return getRightEncoderPosition();
			}
		});

		graphTab.addNumber("l_pos", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return getLeftEncoderPosition();
			}
		});

		graphTab.addNumber("r_vel", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return getRightEncoderRate();
			}
		});

		graphTab.addNumber("l_vel", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return getLeftEncoderRate();
			}
		});

		graphTab.addNumber("l_volts", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return m_leftMaster.getMotorOutputVoltage();
			}
		}).withWidget("Graph");

		graphTab.addNumber("r_volts", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return m_rightMaster.getMotorOutputVoltage();
			}
		}).withWidget("Graph");

		graphTab.addNumber("r_setpoint", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return Constants.AutoConstants.R_CONTROLLER.getSetpoint();
			}
		}).withWidget("Graph");

		graphTab.addNumber("l_setpoint", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return Constants.AutoConstants.L_CONTROLLER.getSetpoint();
			}
		}).withWidget("Graph");
	}

}