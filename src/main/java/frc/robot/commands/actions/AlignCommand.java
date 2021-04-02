/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.actions;

import java.util.Map;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState.States;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.PIDFController;

public class AlignCommand extends CommandBase {
	private TurretSubsystem m_turret;
	private DriveTrainSubsystem m_drive;
	private PIDFController angleController = new PIDFController("Angle", Constants.VisionConstants.kP,
			Constants.VisionConstants.kI, Constants.VisionConstants.kD, 0);

	/**
	 * Creates a new AlignCommand.
	 */
	public AlignCommand(TurretSubsystem m_turret, DriveTrainSubsystem m_drive) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.m_turret = m_turret;
		this.m_drive = m_drive;
		addRequirements(m_turret, m_drive);
		angleController.setTolerance(1, 1);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Limelight.getInstance().turnLightOn();
		System.out.println("STARTING ALIGN COMMAND");
		if (Limelight.getInstance().getPipeIndex() == 0) {
			angleController.setPID(Constants.VisionConstants.kP, Constants.VisionConstants.kI,
					Constants.VisionConstants.kD);
		} else {
			angleController.setPID(Constants.VisionConstants.kP_far, Constants.VisionConstants.kI_far,
					Constants.VisionConstants.kD_far);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Constants.m_RobotState.setState(States.ALIGNING);
		if (Limelight.getInstance().hasValidTarget()) {
			double angle = -angleController.calculate(Limelight.getInstance().getXAngle());
			double output = (angle > 0) ? Constants.VisionConstants.ks + angle : -Constants.VisionConstants.ks + angle;
			m_turret.startTurret(output);
		} else if (!Limelight.getInstance().hasValidTarget()) {
			if (!Limelight.getInstance().hasValidTarget()){
				double xPos = m_drive.getXDisplacement();
				double yPos = m_drive.getYDisplacement();
				double dtAngle = m_drive.getHeading();
				double angleDeError = (180/Math.PI) * Math.atan((Constants.ShooterConstants.targetY - yPos)/(Constants.ShooterConstants.targetX - xPos))+dtAngle;
				double PIDangle = -angleController.calculate(angleDeError);
				double output = (PIDangle>0 ? Constants.VisionConstants.ks + PIDangle : -Constants.VisionConstants.ks + PIDangle);
				m_turret.startTurret(output);
			}
		} else {
			m_turret.stopTurret();
		}		
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_turret.stopTurret();
		angleController.reset();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		
			return false;
	}
}