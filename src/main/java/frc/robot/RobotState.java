/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * Robot State Machine
 */
public class RobotState {

	private static States currentState;

	public static enum States {
		/**
		 * IDLE - The robot is currently not doing anything
		 * 
		 * DRIVE - The robot is not doing anything else but driving
		 * 
		 * AUTO - The robot is in autonomous mode
		 * 
		 * VISION - The robot is currently using vision tracking
		 * 
		 * SHOOTING - The robot is currently trying to shoot a POWER CELL
		 * 
		 * CLIMBING - The robot is currently attempting to/is climbing
		 * 
		 * ROTATION - The robot is currently attempting Rotation Control
		 * 
		 * POSITION - The robot is currently attempting Position Control
		 * 
		 * INTAKE - The robot is currently intaking
		 */
		IDLE(0), DRIVE(1),
		ALIGNING(2), ALIGNED(3),
		REVING(4), REVED(5),
		INTAKING(6);

		private int state;

		/**
		 * A state
		 * 
		 * @param state - The state represented as an integer
		 * @param color - The Color associated with the state
		 */
		private States(int state) {
			this.state = state;
		}

		/**
		 * Get the state as an integer
		 * 
		 * @return The state as an integer
		 */
		public int getState() {
			return state;
		}

		public String getString() {
			switch (state) {
			case 0:
				return "IDLE";
			case 1:
				return "DRIVE";
			case 2:
				return "AlIGNING";
			case 3:
				return "ALIGNED";
			case 4:
				return "REVING";
			case 5:
				return "REVED";
			case 6:
				return "INTAKING";
			default:
				return "IDLE";
			}
		}
	}

	public RobotState(States state) {
		if (state != null)
			currentState = state;
		else
			currentState = States.IDLE;

		Shuffleboard.getTab("Teleop").addString("Robot State", new Supplier<String>() {
			@Override
			public String get() {
				return currentState.getString();
			}
		});
	}

	/**
	 * Get the Robot's current state
	 * 
	 * @return The Robot's current state
	 */
	public States getCurrentState() {
		return currentState;
	}

	/**
	 * Set the current state of the robot
	 * 
	 * @param state - The state the robot should be in
	 */
	public void setState(States state) {
		currentState = state;
	}

}