// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainCommand extends CommandBase {
	private DriveTrainSubsystem drivetrain;
	private boolean slowMode = false;

	/** Creates a new DriveTrainCommand. */
	public DriveTrainCommand(DriveTrainSubsystem drivetrain) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drivetrain.stop();
		drivetrain.calibrateGyro();
		drivetrain.resetGyro();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (RobotContainer.getController().getStickButtonPressed(Hand.kLeft)) slowMode = !slowMode;
		// else if (RobotContainer.getController().getStickButtonPressed(Hand.kRight)) doubleSlowMode = !doubleSlowMode;
		Map<String, Double> sticks = RobotContainer.controller.getSticks();
		drivetrain.driveCartesian(sticks.get("LSX"), sticks.get("LSY"), sticks.get("RSX"), slowMode);

		if (RobotContainer.getController().getAButtonPressed()) drivetrain.resetGyro();

		if (RobotContainer.getController().getStickButtonPressed(Hand.kRight)) RobotContainer.movement = !RobotContainer.movement;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		RobotContainer.drivetrain.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
