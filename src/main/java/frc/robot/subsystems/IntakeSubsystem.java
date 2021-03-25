package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeStates;;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonFX m_intake;
  private Solenoid m_piston;

  private IntakeStates m_intakeState = IntakeStates.STORED;

  public IntakeSubsystem() {
    m_intake = new WPI_TalonFX(Constants.IntakeConstants.INTAKE_ID);
    m_piston = new Solenoid(Constants.IntakeConstants.PISTON_ID); // ???: Piston ID
    // m_intake.setInverted(true); // if inverted (inwards is positive)
    m_intake.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Updates intake status.
   */
  private void updateState() {
    if (m_intake.getSelectedSensorVelocity() != 0) {
      m_intakeState = IntakeStates.INTAKING;
    } else {
      m_intakeState = (m_piston.get()) ? IntakeStates.EXTENDED : IntakeStates.STORED;
    }
  }

  /**
   * Stops the intake.
   */
  public void stopIntake() {
    m_intake.stopMotor();
  }

  /**
   * Extends the intake.
   */
  public void extendIntake() {
    m_piston.set(true);
  }

  /**
   * Retracts the intake.
   */
  public void reset() {
    m_intake.stopMotor();
    m_piston.set(false);
  }

  /**
   * Starts the intake. If the intake is not extended, the intake extends.
   */
  public void startIntake(double triggerVal) {
    m_intake.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_SPEED * triggerVal);
/*     if (m_intakeState == IntakeStates.EXTENDED) {
      m_intake.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_SPEED);
    } else {
      extendIntake();
      m_intake.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_SPEED);
    } */
  }

  /**
   * @return intake state
   */
  public IntakeStates getIntakeState() {
    return m_intakeState;
  }

  @Override
  public void periodic() {
    updateState();
  }
}
