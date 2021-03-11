// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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
    public final static int FALCON_CPR = 2048;

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

        public static final int RIGHT_MASTER = 0;
        public static final int LEFT_MASTER = 3;
        public static final int RIGHT_SLAVE = 4;
        public static final int LEFT_SLAVE = 7;

        public static final double ksVolts = 0.0869 / 10;
        public static final double kvVoltSecondsPerMeter = 2.46 / 10;
        public static final double kaVoltSecondsSquaredPerMeter = 0.185 / 10; // 1.9356652467050692

        public static final double kTrackwidthMeters = Units.inchesToMeters(22);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static double kP = 0.01; // 7.43;
        public static double kI = 0; // 0.01; // 7.43;
        public static double kD = 0; // 0.01; // 7.43;

    }

    public final static class AutoConstants {
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double kMaxSpeedMetersPerSecond = .8;
        public static final double kMaxAccelerationMetersPerSecondSquared = .5;
        public static final PIDController L_CONTROLLER = new PIDController(DriveConstants.kP, DriveConstants.kI,
                DriveConstants.kD);
        public static final PIDController R_CONTROLLER = new PIDController(DriveConstants.kP, DriveConstants.kD,
                DriveConstants.kD);

    }

    public final static class ShooterConstants {
        public static final int HOOD_ID = -1; // placeholder
        public static final int SHOOTER_ID = -1; // placeholder
        public static final int TURRET_ID = -1; // placeholder
        public static final int SHOOTER_INTAKE_ID = -1; // placeholder

        public static final double HOOD_GEAR_RATIO = 1 / 37.7778;

        public static double TURRET_kP = 0;
        public static double TURRET_kI = 0;
        public static double TURRET_kD = 0;
    }
}
