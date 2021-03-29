/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.utils.PIDFController;

public class TurnToDegrees extends CommandBase {
	private DriveTrainSubsystem m_drive;
	private double degrees;

	private PIDFController angleController = new PIDFController("Angle", .018, Constants.VisionConstants.kI, .018 / 3.6,
			0.05);

	/**
	 * Creates a new TurnToDegrees.
	 */
	public TurnToDegrees(DriveTrainSubsystem drive, double d) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_drive = drive;
		degrees = d;

		angleController.setTolerance(3, 3);
		addRequirements(m_drive);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		angleController.setSetpoint(m_drive.getHeading() + degrees);
		System.out.println("STARTED DEGREE TURN");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
        System.out.println(-angleController.getPositionError());
        //ngl this is probably wrong will fix later
		m_drive.driveCartesian(0, 0, -angleController.calculate(m_drive.getHeading()));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("ENDED DEGREE TURN");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return angleController.atSetpoint();
	}
}