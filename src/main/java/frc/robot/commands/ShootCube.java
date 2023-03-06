package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;

public class ShootCube extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private IntakeSystem m_IntakeSystem;
    private boolean shoot;
    private int speedLevel;  
  public ShootCube(IntakeSystem intake, int m_speedLevel) {
    m_IntakeSystem = intake;
    speedLevel=m_speedLevel;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSystem.ShootCube(speedLevel);
    shoot=false;
  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    if(m_IntakeSystem.AtShootSpeed()) shoot=true;
    if(shoot) m_IntakeSystem.injectCube();
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
    return false;
  }
}

