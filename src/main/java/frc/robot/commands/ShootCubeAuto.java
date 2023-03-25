package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;

public class ShootCubeAuto extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private IntakeSystem m_IntakeSystem;
    private int speedLevel; 
    private double timeStart;
    private double eject_delay;
    private double shoot_delay;
  public ShootCubeAuto(IntakeSystem intake, int m_speedLevel) {
    m_IntakeSystem = intake;
    speedLevel=m_speedLevel;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeStart=Timer.getFPGATimestamp();
    m_IntakeSystem.ShootCube(speedLevel);
    if (speedLevel == 2) {
      eject_delay = 0.75;
      shoot_delay = 0.75;
    } else if (speedLevel == 3){
      eject_delay = 0.25;
      shoot_delay = 1.0;
    } else if (speedLevel == 4){
      eject_delay = 0.25;
      shoot_delay = 1.25;
    } else {
      eject_delay = 0.25;
      shoot_delay = 1.25;
    }
  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    
    if(Timer.getFPGATimestamp()>timeStart+eject_delay) m_IntakeSystem.injectCube(speedLevel);
    SmartDashboard.putNumber("Top Conveyor RPM", m_IntakeSystem.getTopConveyorSpeed()*600/2048);
    SmartDashboard.putNumber("Shooter Top RPM", m_IntakeSystem.getTopShooterRPM() );
    SmartDashboard.putNumber("Shooter Bot RPM", m_IntakeSystem.getBottomShooterRPM() );
    SmartDashboard.putNumber("BotConveyor CLE", m_IntakeSystem.getShooterCLE());
  } 

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.StopMotors();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp()>timeStart+shoot_delay;
  }
}

