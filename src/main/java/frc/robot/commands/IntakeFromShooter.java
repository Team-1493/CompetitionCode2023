package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;

public class IntakeFromShooter extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private IntakeSystem m_IntakeSystem;
    private double timeStart;
    private double deltaTime=0.2;
    private boolean hasCube=false;  

  public IntakeFromShooter(IntakeSystem intake) {
    m_IntakeSystem = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // deltaTime=SmartDashboard.getNumber("Rev Shooter dt", deltaTime);
    m_IntakeSystem.reverseIntake();
    hasCube=false;
  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
   
  } 

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(m_IntakeSystem.HasCube() && !hasCube){
      hasCube=true;
      timeStart=Timer.getFPGATimestamp();
    
    }
    if (hasCube && Timer.getFPGATimestamp()-timeStart>deltaTime)
      return true;  
    else 
      return false; 

  }

    
}
