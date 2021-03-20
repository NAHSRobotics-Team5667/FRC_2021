package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexStates;

public class IndexSubsystem extends SubsystemBase {
  private WPI_TalonFX m_index;
  private IndexStates m_indexState = IndexStates.IDLE;

  /** Creates a new IndexSubsystem. */
  public IndexSubsystem() {
    m_index = new WPI_TalonFX(Constants.IndexConstants.INDEX_ID);
    // m_index.setInverted(true); // if inverted (clockwise is positive)
    m_index.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Updates index state.
   */
  private void updateState() {
    m_indexState = (m_index.getSelectedSensorVelocity() != 0) ? IndexStates.POWERED : IndexStates.IDLE;
  }

  /**
   * Stops index rotation.
   */
  public void stopIndex() {
    m_index.stopMotor();
  }

  /**
   * Starts index rotation.
   */
  public void startIndex() {
    m_index.set(ControlMode.PercentOutput, Constants.IndexConstants.INDEX_SPEED);
  }

  /**
   * @return index state.
   */
  public IndexStates getIndexState() {
    return m_indexState;
  }

  @Override
  public void periodic() {
    updateState();
  }
}
