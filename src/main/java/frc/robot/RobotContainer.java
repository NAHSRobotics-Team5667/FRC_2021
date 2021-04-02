// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.FileSystem;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.shooter.TurretCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.actions.AlignCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Controller;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.RunPath;
import frc.robot.Constants.PATHS;
import frc.robot.Constants.PATHS.PathWeaver;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static Controller controller = new Controller(0);
	public static DriveTrainSubsystem drivetrain;
	public static IndexSubsystem m_index;
	public static IntakeSubsystem m_intake;
	public static ShooterSubsystem m_shooter;
	public static TurretSubsystem m_turret;
	public static PowerDistributionPanel panel = new PowerDistributionPanel(0);

	private Trajectory[] paths = new Trajectory[] { PATHS.PathWeaver.getTrajectory("FAR_TRENCH"),
			PATHS.PathWeaver.getTrajectory("FAR_RENDEVOUS"), PATHS.PathWeaver.getTrajectory("MIDDLE_TRENCH"),
			PATHS.PathWeaver.getTrajectory("MIDDLE_RENDEVOUS"), PATHS.PathWeaver.getTrajectory("CLOSE_TRENCH"),
			PATHS.PathWeaver.getTrajectory("CLOSE_RENDEVOUS"), PATHS.PathWeaver.getTrajectory("BALL_THIEF"), null,
			PATHS.PathWeaver.getTrajectory("MIDDLE_TRENCH_SIDE"), null, PATHS.STRAIGHT_TRAJECTORY_2M,
			PATHS.S_TRAJECTORY, PathWeaver.getTrajectory("slalom") };

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		drivetrain = new DriveTrainSubsystem(new AHRS(SPI.Port.kMXP), new WPI_TalonFX(Constants.DriveConstants.FR), new WPI_TalonFX(Constants.DriveConstants.FL), new WPI_TalonFX(Constants.DriveConstants.RR), new WPI_TalonFX(Constants.DriveConstants.RL));
		// XXX: these are placeholders!!
		m_shooter = new ShooterSubsystem(0.0, 0.0);
		m_intake = new IntakeSubsystem();
		m_index = new IndexSubsystem();
		m_turret = new TurretSubsystem(-20);
		configureButtonBindings();
		//Set default commands
		drivetrain.setDefaultCommand(new DriveTrainCommand());
		m_index.setDefaultCommand(new IndexCommand(m_index));
		m_intake.setDefaultCommand(new IntakeCommand(m_intake));
		m_shooter.setDefaultCommand(new ShooterCommand(m_shooter));
		m_turret.setDefaultCommand(new TurretCommand(m_turret));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		Button b = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_B_PORT);
		Button y = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_Y_PORT);

		Button left_stick = new JoystickButton(getController(), Constants.ControllerConstants.S_LEFT);
		Button right_stick = new JoystickButton(getController(), Constants.ControllerConstants.S_RIGHT);

		Button start = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_START_PORT);
		Button menu = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_MENU_PORT);

		Button right_bumper = new JoystickButton(getController(), Constants.ControllerConstants.BUMPER_RIGHT_PORT);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public static Controller getController() {
		return controller;
	}
	public Command getAutonomousCommand(int selection) {
		// drivetrain.resetOdometry(paths[selection].getInitialPose());
		// return RunPath.getCommand(paths[selection], drivetrain, false).andThen(new RunCommand(drivetrain::stop));

		return new AlignCommand(m_turret, drivetrain);
	}
	
	public void setNeutralMode(NeutralMode mode){
		drivetrain.setNeutralMode(mode);
	}	
	public void feedMotorSafety() {
		drivetrain.feedMotorSafety();
	}
}
