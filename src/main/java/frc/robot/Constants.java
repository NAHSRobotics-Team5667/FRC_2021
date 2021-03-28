// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.MecanumDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int FALCON_CPR = 2048;

    public final static class ControllerConstants {
        public static final int CONTROLLER_PORT = 0; // Controller port

        // Sticks
        public static final int S_RIGHT_X_PORT = 4; // Right stick x
        public static final int S_RIGHT_Y_PORT = 5; // Right stick y
        public static final int S_LEFT_X_PORT = 0; // Left stick x
        public static final int S_LEFT_Y_PORT = 1; // Left stick y

        public static final int S_LEFT = 9; // Left stick button
        public static final int S_RIGHT = 10; // Right stick button

        // Triggers
        public static final int TRIGGER_RIGHT_PORT = 3; // Right trigger
        public static final int TRIGGER_LEFT_PORT = 2; // Left trigger

        // Bumpers
        public static final int BUMPER_RIGHT_PORT = 6; // Right bumper
        public static final int BUMPER_LEFT_PORT = 5; // Left bumper

        // Buttons
        public static final int BUTTON_A_PORT = 1; // A Button
        public static final int BUTTON_B_PORT = 2; // B Button
        public static final int BUTTON_X_PORT = 3; // X Button
        public static final int BUTTON_Y_PORT = 4; // Y Button

        // Special buttons
        public static final int BUTTON_MENU_PORT = 8; // Menu Button
        public static final int BUTTON_START_PORT = 7; // Start button

    }
    public final static class DriveConstants {
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER * Math.PI;
        public static final double ENCODER_EDGES_PER_REV = 21934;
        public static final double GEAR_RATIO = 10.71;
        public static double MAG = -1;
        public static final double ENCODER_CONSTANT = MAG * (1 / ENCODER_EDGES_PER_REV) * WHEEL_DIAMETER * Math.PI;
        public static final boolean kGyroReversed = true;

        public static final double MAX_SPEED_TELE = 3.25;
        public static final double MAX_ANGULAR_VEL = 320;

        public static final int FR = 0;
        public static final int FL = 3;
        public static final int RR = 4;
        public static final int RL = 7;

        public static final double ksVolts = 0.0869 / 10;
        public static final double kvVoltSecondsPerMeter = 2.46 / 10;
        public static final double kaVoltSecondsSquaredPerMeter = 0.185 / 10; // 1.9356652467050692

        public static final double kTrackwidthMeters = Units.inchesToMeters(20.36);
        public static final double kTracklengthMeters = Units.inchesToMeters(20.36);
        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(new Translation2d(kTrackwidthMeters, kTracklengthMeters),
        new Translation2d(kTrackwidthMeters, -kTracklengthMeters), new Translation2d(-kTrackwidthMeters, kTracklengthMeters), new Translation2d(-kTrackwidthMeters, -kTracklengthMeters));

        public static double startY = 1.0;
        public static double startX = 13.5;

        public static double kP = 0.01; // 7.43;
        public static double kI = 0; // 0.01; // 7.43;
        public static double kD = 0; // 0.01; // 7.43;

    }

    public final static class AutoConstants {
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double kMaxSpeedMetersPerSecond = .8;
        public static final double kMaxAccelerationMetersPerSecondSquared = .5;
        public static final PIDController xController = new PIDController(DriveConstants.kP, DriveConstants.kI,
                DriveConstants.kD);
        public static final PIDController yController = new PIDController(DriveConstants.kP, DriveConstants.kI,
                DriveConstants.kD);
        public static final PIDController FL_CONTROLLER = new PIDController(DriveConstants.kP, DriveConstants.kI,
                DriveConstants.kD);
        public static final PIDController FR_CONTROLLER = new PIDController(DriveConstants.kP, DriveConstants.kD,
                DriveConstants.kD);
                public static final PIDController RL_CONTROLLER = new PIDController(DriveConstants.kP, DriveConstants.kI,
                DriveConstants.kD);
        public static final PIDController RR_CONTROLLER = new PIDController(DriveConstants.kP, DriveConstants.kD,
                DriveConstants.kD);
        public static final Constraints theta_constraints = new Constraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        public static final ProfiledPIDController THET_CONTROLLER = new ProfiledPIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD, theta_constraints);

    }
    public final static class PATHS {
        public static final MecanumDriveKinematicsConstraint autoVoltageConstraint = new MecanumDriveKinematicsConstraint(
               Constants.DriveConstants.kDriveKinematics, Constants.AutoConstants.kMaxSpeedMetersPerSecond);

        // Create config for trajectory
        public static final TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow. All units in meters.
        public static final Trajectory S_TRAJECTORY = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);

        public static final Trajectory STRAIGHT_TRAJECTORY_2M = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints
                List.of(new Translation2d(1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                // Pass config
                config);

        public static final Trajectory OFF_LINE = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints
                List.of(new Translation2d(1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                // Pass config
                config);

        public static final Trajectory TRENCH_LINE = TrajectoryGenerator.generateTrajectory(
                // Start
                new Pose2d(5.2, -0.7, new Rotation2d(0)),
                // Pass through balls
                List.of(new Translation2d(5.885, -0.7)),
                // End at the end of the color wheel
                new Pose2d(6.57, -0.7, new Rotation2d(0)), config);

        public static final Trajectory SIDE_TRENCH = TrajectoryGenerator.generateTrajectory(
                // Start
                new Pose2d(6.244, -1.463, new Rotation2d(90)),
                // Pass through balls
                List.of(new Translation2d(6.244, -1.1)),
                // End at the end of the color wheel
                new Pose2d(6.244, -1, new Rotation2d(90)), config);

        public static final Trajectory SIDE_FORWARD = PathWeaver.getTrajectory("TRENCH_LINE");

        public static final class PathWeaver {
            public static Trajectory getTrajectory(String path) {
                // try {
                //     return TrajectoryUtil
                //             .fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/" + path + ".wpilib.json"));
                // } catch (Exception e) {
                //     System.out.println("CANNOT READ Trajectory - " + path);
                //     System.out.println("WITH ERROR: " + e.toString());
                     return null;
                // }
            }
        }
    }


    public static final class ShooterConstants {
        public static final int HOOD_ID = -1; // placeholder
        public static final int SHOOTER_ID = -1; // placeholder
        public static final int TURRET_ID = -1; // placeholder
        public static final int SHOOTER_INTAKE_ID = -1; // placeholder

        public static final double HOOD_VOLTAGE = 6.0;
        public static final double SHOOTER_VOLTAGE = 10.0;
        public static final double INTAKE_VOLTAGE = 8.0;
        public static final double TURRET_VOLTAGE = 6.0;

        public static final double HOOD_GEAR_RATIO = 1 / 37.7777777777777777777777777777;
        public static final double TURRET_GEAR_RATIO = 1 / 53.3333333333333333333333333333;

        public static double TURRET_kP = 0.02; // placeholder
        public static double TURRET_kI = 0; // placeholder
        public static double TURRET_kD = 0; // placeholder
    }

    public enum ShooterStates {
        IDLE,
        SEARCHING,
        SHOOTING
    }

    public static final class IntakeConstants {
        public static final int INTAKE_ID = 5;
        public static final int PISTON_ID = 1; // placeholder

        public static final double INTAKE_SPEED = .45; // placeholder
    }

    public enum IntakeStates {
        STORED,
        EXTENDED,
        INTAKING
    }

    public static final class IndexConstants {
        public static final int INDEX_ID = 2;

        public static final double INDEX_SPEED = 0.1;
    }

    public enum IndexStates {
        IDLE,
        POWERED
    }
}
