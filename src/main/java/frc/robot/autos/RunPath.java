/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autos;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * Get a Ramsete Command that drives a path
 */
public class RunPath {
    public static MecanumControllerCommand getCommand(Trajectory path, DriveTrainSubsystem drive, boolean isReverse) {
        if (isReverse)
            drive.reverseEncoders();
        drive.setNeutralMode(NeutralMode.Brake);
        return new MecanumControllerCommand(path, drive::getPose,
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, AutoConstants.xController, AutoConstants.yController, AutoConstants.THET_CONTROLLER, AutoConstants.kMaxSpeedMetersPerSecond,
                 AutoConstants.FL_CONTROLLER, AutoConstants.RL_CONTROLLER, AutoConstants.FR_CONTROLLER, AutoConstants.RR_CONTROLLER, drive::getWheelSpeeds,
                // RamseteCommand passes volts to the callback
                (drive::driveVoltage), drive);
    }
}