// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class DriveTrainPlayback extends CommandBase {
	private DriveTrainSubsystem drivetrain;
	private boolean slowMode = false;
	private File file;
	BufferedReader br;
	StringBuilder sb;
	String line;
	double inputLSX;
	double inputLSY;
	double inputRSX;
	private final double tStep = 0.005;


	/** Creates a new DriveTrainCommand. */
	public DriveTrainPlayback(DriveTrainSubsystem drivetrain, String command) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
		try {
			br = new BufferedReader(new FileReader("/copypaths/" + command + ".txt"));
		    sb = new StringBuilder();
			line = br.readLine();
	}
		catch(IOException e){
			e.printStackTrace();
		}
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
		while(line!=null){
		int prev = -1;
		int curr;
		curr = line.indexOf(",", prev+1);
		inputLSX = Double.parseDouble(line.substring(prev+1, curr));
		prev = curr;
		curr = line.indexOf(",", prev+1);
		inputLSY = Double.parseDouble(line.substring(prev+1, curr));
		prev = curr;
		curr = line.indexOf(",", prev+1);
		inputRSX = Double.parseDouble(line.substring(prev+1, curr));
		drivetrain.driveCartesian(inputLSX, inputLSY, inputRSX, slowMode);
		try{
		line = br.readLine();
		}
		catch(IOException e){
			e.printStackTrace();
		}
		Timer.delay(tStep);
		if (RobotContainer.getController().getAButtonPressed()) drivetrain.resetGyro();

		if (RobotContainer.getController().getStickButtonPressed(Hand.kRight)) RobotContainer.movement = !RobotContainer.movement;
		if(RobotContainer.getController().getDPad() == 0){
			try{
				br.close();
		}
		catch(IOException e){
			e.printStackTrace();
		}
	}
}

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
