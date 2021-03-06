// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import edu.wpi.first.wpilibj.Timer;

public class DifferentialDriveCopy extends CommandBase {
	private DifferentialDriveSubsystem drivetrain;
	private boolean open = true;
	private boolean slowMode = true;
	File file;
	BufferedWriter bw;
	Timer timer;
	private final double tStep = 0.005;

	/** Creates a new DriveTrainCommand. */
	public DifferentialDriveCopy(DifferentialDriveSubsystem drivetrain) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
		this.timer = new Timer();

	// 	try{
	// 	this.file = new File("/home/lvuser/main/deploy/output/" + pathname +".txt");
	// 	this.bw = new BufferedWriter(new FileWriter(file));
	// 	System.out.println("accessing the file work");
	// 	}
	// catch(IOException e){
	// 	e.printStackTrace();
	// }
	}
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drivetrain.stop();
		drivetrain.calibrateGyro();
		//drivetrain.resetGyro();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (RobotContainer.getController().getStickButtonPressed(Hand.kLeft)){ slowMode = !slowMode;}
		// else if (RobotContainer.getController().getStickButtonPressed(Hand.kRight)) doubleSlowMode = !doubleSlowMode;
		Map<String, Double> sticks = RobotContainer.controller.getSticks();
		String outstring = "0,0,\n";
		if(open){
		outstring = sticks.get("LSY").toString() + "," + sticks.get("RSX").toString() + ",\n";
		System.out.print(outstring);
	// 	if(bw!=null){
	// 	try {
	// 		bw.write(outstring);
	// 	} catch (IOException e) {
	// 		// TODO Auto-generated catch block
	// 		e.printStackTrace();
	// 	}
	// }
	// 	else{
	// 	}
}
		drivetrain.drive(0.7*sticks.get("LSY"), 0.7*sticks.get("RSX"),
		RobotContainer.getController().getStickButtonPressed(RobotContainer.getController().getLeftHand()));

		if (RobotContainer.getController().getAButtonPressed()){
					 drivetrain.resetGyro();
				}

		if (RobotContainer.getController().getStickButtonPressed(Hand.kRight)) RobotContainer.movement = !RobotContainer.movement;
		//Timer.delay(tStep);
		if(RobotContainer.getController().getDPad() == 180){
			open = false;
			try{
			bw.close();
			}
			catch(IOException e){
				e.printStackTrace();
			}
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

