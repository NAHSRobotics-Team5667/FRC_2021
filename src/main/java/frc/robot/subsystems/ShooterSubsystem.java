package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterStates;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ShooterSubsystem extends SubsystemBase {
  // TODO: add turret integration
  private WPI_TalonFX m_hood, m_shooter, m_shooterIntake;

  private double initialHoodAngle;
  private double hoodAngle;

  private double initialTurretAngle;
  private double turretAngle;

  private ShooterStates m_shooterState = ShooterStates.IDLE;

  /**
   * @param hoodAngle   The initial angle of the hood.
   * @param turretAngle The intiangle of the turret.
   */
  public ShooterSubsystem(double hoodAngle, double turretAngle) {
    m_hood = new WPI_TalonFX(Constants.ShooterConstants.HOOD_ID);
    m_shooter = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_ID);
    m_shooterIntake = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_INTAKE_ID);
    // m_hood.setInverted(true); // positive makes the angle larger, negative makes the angle smaller
    // m_shooter.setInverted(true); // positive shoots power cells
    m_hood.setNeutralMode(NeutralMode.Brake);
    m_shooter.setNeutralMode(NeutralMode.Coast);
    m_shooterIntake.setNeutralMode(NeutralMode.Brake);
    m_hood.setVoltage(6); // requires less voltage, may need more idk
    m_shooter.setVoltage(10); // requires more voltage to launch powercells idk
    m_shooterIntake.setVoltage(8); // requires less voltage than shooter but more than turret

    this.initialHoodAngle = hoodAngle;
    this.hoodAngle = hoodAngle;
  }

  public void stopHood() {
    m_hood.stopMotor();
  }

  public void stopShooter() {
    m_shooterIntake.stopMotor();
    m_shooter.stopMotor();
  }

  public double getShooterRpm() {
    return m_shooter.getSelectedSensorVelocity();
  }

  /**
   * @param tickDifference difference in ticks for the hood motor. Can be obtained by calling the degreesToTicks() function.
   * @param percentOutput  percentage of voltage to output to the motor.
   */
  public void setHood(double tickDifference, double percentOutput) {
    double targetTicks = m_hood.getSelectedSensorPosition() + tickDifference;

    if (m_hood.getSelectedSensorPosition() < targetTicks) m_hood.set(ControlMode.PercentOutput, percentOutput);
    else if (m_hood.getSelectedSensorPosition() > targetTicks) m_hood.set(ControlMode.PercentOutput, -percentOutput);
  }

  /**
   * @param distance normal distance from inner power port
   */
  public double distanceToDegrees(double distance) {
    return -1; // placeholder, requires data from testing
  }

  /**
   * @param endDegrees     end degree position of the hood.
   * @param initialDegrees initial degree position of the hood.
   * @return               Tick difference for the Falcon 500.
   */
  public double degreesToHoodTicks(double endDegrees, double initialDegrees) {
    return ((Constants.FALCON_CPR * (1 / Constants.ShooterConstants.HOOD_GEAR_RATIO)) * (endDegrees - initialDegrees)) / 360;
  }

  public void startShooter() {
    m_shooterIntake.set(ControlMode.PercentOutput, 1); // set shooter-intake to full speed
    m_shooter.set(ControlMode.PercentOutput, 1); // set shooter to full speed
  }

  /**
   * @param hoodTicks encoder count from Falcon 500.
   */
  private void updateHoodAngle(double hoodTicks) {
    hoodAngle = ((hoodTicks * 360) / (Constants.FALCON_CPR * (1 / Constants.ShooterConstants.HOOD_GEAR_RATIO))) + initialHoodAngle;
  }

  /**
   * @return Angle of the powercell in relation to the ground when it exits the shooter.
   */
  public double getHoodAngle() {
    return hoodAngle;
  }

  /**
   * @return shooter state.
   */
  public ShooterStates getShooterState() {
    return m_shooterState;
  }

  @Override
  public void periodic() {
    updateHoodAngle(m_hood.getSelectedSensorPosition());
  }
}
