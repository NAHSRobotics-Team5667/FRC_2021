package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  private double timeout; // time to run the shooter for
  private double initTime;

  private ShooterSubsystem m_shooter;

  /** 
   * Creates a new ShooterCommand. Use when shooting in teleoperated period.
   * 
   * @param m_shooter The shooter subsystem.
  */
  public ShooterCommand(ShooterSubsystem m_shooter) {
    addRequirements(m_shooter);
  }

  /**
   * Creates a new ShooterCommand. Use when shooting autonomously.
   * 
   * @param timeout period of time motor will be powered for (seconds).
   * @param m_shooter the shooter subsystem.
   */
  public ShooterCommand(double timeout, ShooterSubsystem m_shooter) {
    this.timeout = timeout;
    this.m_shooter = m_shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - initTime <= timeout) {
      if(RobotContainer.controller.getRightTrigger()>0.05){
      m_shooter.startShooter();
    }
  }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return (false); // end the command when difference is greater than timeout
  }
}
