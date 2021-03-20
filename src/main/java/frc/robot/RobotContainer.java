// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Controller;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.RunPath;
import frc.robot.Constants.PATHS;


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
	public static ShooterSubsystem m_shooter;
	public static PowerDistributionPanel panel = new PowerDistributionPanel(0);

	private Trajectory[] paths = new Trajectory[] { PATHS.PathWeaver.getTrajectory("FAR_TRENCH"),
			PATHS.PathWeaver.getTrajectory("FAR_RENDEVOUS"), PATHS.PathWeaver.getTrajectory("MIDDLE_TRENCH"),
			PATHS.PathWeaver.getTrajectory("MIDDLE_RENDEVOUS"), PATHS.PathWeaver.getTrajectory("CLOSE_TRENCH"),
			PATHS.PathWeaver.getTrajectory("CLOSE_RENDEVOUS"), PATHS.PathWeaver.getTrajectory("BALL_THIEF"), null,
			PATHS.PathWeaver.getTrajectory("MIDDLE_TRENCH_SIDE"), null, PATHS.STRAIGHT_TRAJECTORY_2M,
			PATHS.S_TRAJECTORY };
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		drivetrain = new DriveTrainSubsystem();
		m_index = new IndexSubsystem();
		configureButtonBindings();
		//Set default commands
		drivetrain.setDefaultCommand(new DriveTrainCommand());
		m_index.setDefaultCommand(new IndexCommand(m_index, m_shooter));

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
			drivetrain.resetOdometry(paths[selection].getInitialPose());
			return RunPath.getCommand(paths[selection], drivetrain, false).andThen(new RunCommand(drivetrain::stop));
		
		
		}
	
	public void setNeutralMode(NeutralMode mode){
		drivetrain.setNeutralMode(mode);
	}	
	public void feedMotorSafety() {
		drivetrain.feedMotorSafety();
	}
}
