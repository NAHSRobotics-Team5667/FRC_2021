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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DifferentialDriveCommand;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.DriveTrainCopy;
import frc.robot.commands.DriveTrainPlayback;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.shooter.TurretCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DifferentialDriveSubsystem;
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
import frc.robot.autos.RunPathDiff;
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

	public static DifferentialDriveSubsystem m_diffDrive;

	public static boolean movement = true;

	private Trajectory[] paths = new Trajectory[] { PATHS.STRAIGHT_TRAJECTORY_2M,
			PATHS.S_TRAJECTORY, PathWeaver.getTrajectory("sqr"),PathWeaver.getTrajectory("slalomreverse"), PathWeaver.getTrajectory("slalom"),
			PathWeaver.getTrajectory("barrel_race"), PathWeaver.getTrajectory("bounce"),
			PathWeaver.getTrajectory("bounce2"), PathWeaver.getTrajectory("bounce3"),
			PathWeaver.getTrajectory("bounce4"), PathWeaver.getTrajectory("gs1"),
			PathWeaver.getTrajectory("gs2"), PathWeaver.getTrajectory("straight") };

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		drivetrain = new DriveTrainSubsystem(new AHRS(SPI.Port.kMXP), new WPI_TalonFX(Constants.DriveConstants.FR), new WPI_TalonFX(Constants.DriveConstants.FL), new WPI_TalonFX(Constants.DriveConstants.RR), new WPI_TalonFX(Constants.DriveConstants.RL));
		// XXX: these are placeholders!!
		m_shooter = new ShooterSubsystem(0.0, 0.0);
		m_intake = new IntakeSubsystem();
		m_index = new IndexSubsystem();
		m_turret = new TurretSubsystem(0);
		//m_diffDrive = new DifferentialDriveSubsystem(new AHRS(SPI.Port.kMXP), new WPI_TalonFX(Constants.DriveConstants.FR), new WPI_TalonFX(Constants.DriveConstants.FL), new WPI_TalonFX(Constants.DriveConstants.RR), new WPI_TalonFX(Constants.DriveConstants.RL));
		configureButtonBindings();
		//Set default commands
		//m_diffDrive.setDefaultCommand(new DifferentialDriveCommand(m_diffDrive));
		drivetrain.setDefaultCommand(new DriveTrainCommand(drivetrain));
		//drivetrain.setDefaultCommand(new DriveTrainPlayback(drivetrain, "firsttest"));
		//drivetrain.setDefaultCommand(new DriveTrainCopy(drivetrain, "firsttest"));
		m_index.setDefaultCommand(new IndexCommand(m_index));
		m_intake.setDefaultCommand(new IntakeCommand(m_intake));
		m_shooter.setDefaultCommand(new ShooterCommand(m_shooter));
		m_turret.setDefaultCommand(new AlignCommand(m_turret));
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
	public Command getAutonomousCommand(int selection) { // if selection == 4 call bounce sequential command
		// if (selection != 4) {
		// 	m_diffDrive.resetOdometry(paths[selection].getInitialPose());
		// 	// return RunPath.getCommand(paths[selection], drivetrain, false).andThen(new RunCommand(drivetrain::stop));
		// 	return RunPathDiff.getCommand(paths[selection], m_diffDrive, false).andThen(new RunCommand(m_diffDrive::stop));
		// } else {
		// 	return new SequentialCommandGroup(new Command[] {
		// 		// RunPath.getCommand(paths[4], drivetrain, false), 
		// 		// RunPath.getCommand(paths[5], drivetrain, true),
		// 		// RunPath.getCommand(paths[6], drivetrain, false),
		// 		// RunPath.getCommand(paths[7], drivetrain, true)
		// 		RunPathDiff.getCommand(paths[4], m_diffDrive, false), 
		// 		RunPathDiff.getCommand(paths[5], m_diffDrive, true),
		// 		RunPathDiff.getCommand(paths[6], m_diffDrive, false),
		// 		RunPathDiff.getCommand(paths[7], m_diffDrive, true)
		// 	});
		// }

		m_diffDrive.resetOdometry(paths[selection].getInitialPose());
		// return RunPath.getCommand(paths[selection], drivetrain, false).andThen(new RunCommand(drivetrain::stop));
		return RunPathDiff.getCommand(paths[selection], m_diffDrive, false).andThen(new RunCommand(m_diffDrive::stop));

		// return new AlignCommand(m_turret);
	}
	
	public void setNeutralMode(NeutralMode mode){
		// drivetrain.setNeutralMode(mode);
		drivetrain.setNeutralMode(mode);;
	}	
	public void feedMotorSafety() {
		// drivetrain.feedMotorSafety();
		drivetrain.feedMotorSafety();;
	}
}
