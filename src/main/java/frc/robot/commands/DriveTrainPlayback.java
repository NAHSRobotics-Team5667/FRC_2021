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
	private final String[] bs = {"0.0,0.0,0.16535432636737823", "0.0,0.0,0.18897637724876404", "0.0,0.0,0.21259842813014984", "0.0,0.0,0.23622047901153564", "0.0,0.0,0.24409449100494385", "0.0,0.0,0.25196850299835205", "0.0,0.0,0.25984251499176025", "0.0,0.0,0.25984251499176025", "0.0,0.0,0.26771652698516846", "0.0,0.0,0.26771652698516846", "0.0,0.0,0.26771652698516846", "0.0,0.0,0.26771652698516846", 
	"0.0,0.0,0.26771652698516846", "0.0,0.0,0.26771652698516846", "0.0,0.0,0.26771652698516846", "0.0,0.0,0.26771652698516846", "0.0,0.0,0.25196850299835205", "0.0,0.0,0.18110236525535583", "0.0,0.0,0.14173229038715363", "0.0,0.0,0.12598425149917603", "0.0,0.0,0.11023622006177902", "0.0,0.0,0.11023622006177902", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", 
	"0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", 
	"0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0"
, "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.0,0.0,0.0", "0.375,0.0,0.0", "0.4375,0.1328125,0.0", "0.7421875,0.2109375,0.0"
	, "1.0,0.2578125,0.0", "1.0,0.2578125,0.0", "1.0,0.3125,0.0", "1.0,0.3203125,0.0", "1.0,0.328125,0.0", "1.0,0.3203125,0.0", "1.0,0.3046875,0.0", "1.0,0.3046875,0.0", "1.0,0.3046875,0.0",
	"1.0,0.2890625,0.0", "1.0,0.2890625,0.0", "1.0,0.2890625,0.0", "1.0,0.2890625,0.0", "1.0,0.2734375,0.0", "1.0,0.265625,0.0", "1.0,0.2421875,0.0", "1.0,0.2421875,0.0", "1.0,0.2265625,0.0", 
	"1.0,0.21875,0.0", "1.0,0.21875,0.0", "1.0,0.21875,0.0", "1.0,0.21875,0.0", "1.0,0.21875,0.0", "1.0,0.2109375,0.0", "1.0,0.2109375,0.0", "1.0,0.2109375,0.0", "1.0,0.2265625,0.0", "1.0,0.234375,0.0", "1.0,0.2421875,0.0", "1.0,0.25,0.0", "1.0,0.2578125,0.0", "1.0,0.265625,0.0", "1.0,0.2734375,0.0", "1.0,0.2734375,0.0", "1.0,0.28125,0.0", "1.0,0.28125,0.0", "1.0,0.28125,0.0", "1.0,0.28125,0.0"
, "1.0,0.28125,0.0", "1.0,0.234375,0.0", "1.0,0.0,0.0", "1.0,0.0,0.0", "0.1640625,0.0,0.0", "0.1640625,0.0,0.0"};
	int gajraare = bs.length;


	/** Creates a new DriveTrainCommand. */
	public DriveTrainPlayback(DriveTrainSubsystem drivetrain, String command) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
	// 	try {
	// 		br = new BufferedReader(new FileReader(command + ".txt"));
	// 	    sb = new StringBuilder();
	// 		line = br.readLine();
	// }
	// 	catch(IOException e){
	// 		e.printStackTrace();
	// 	}
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
		for(int j = 0; j<gajraare; j++){
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
		}
		
		Timer.delay(tStep);
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
