package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    this.m_shooter = m_shooter;
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
    m_shooter.stopHood();
    initTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    m_shooter.startShooter(Constants.ShooterConstants.SHOOTER_SPEED);
    double frac = RobotContainer.controller.getRightTrigger();
    if(RobotContainer.controller.getRightTrigger()>0){
      m_shooter.startShooterIntake();
      System.out.println(frac);
    }else{
      m_shooter.stopShooterIntake();
    }

    if (RobotContainer.controller.getXButton()) {
      m_shooter.startHood(Constants.ShooterConstants.HOOD_SPEED);
      System.out.println("gotY");
    } else if(RobotContainer.controller.getBButton()){
      m_shooter.startHood(-Constants.ShooterConstants.HOOD_SPEED);
    } else{
      m_shooter.stopHood();
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_shooter.stopHood();
  }

  @Override
  public boolean isFinished() {
    return (false); // end the command when difference is greater than timeout
  }
}
