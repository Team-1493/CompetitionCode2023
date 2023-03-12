package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;

public class ArmOverCone extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
 
    private ArmSubsystem m_ArmSubsystem;
    private IntakeSystem m_IntakeSystem;
    double time1=0;
    boolean  flag1=false,flag2=false;
  public ArmOverCone(ArmSubsystem arm,IntakeSystem intake) {
    m_ArmSubsystem = arm;
    m_IntakeSystem = intake;
    addRequirements(arm,intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmSubsystem.resetIntegralAccumulator();  
    m_ArmSubsystem.setMagicFast();
    
  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    m_ArmSubsystem.motorActive=true;

    if(m_ArmSubsystem.getCounts()<1400 &&!flag1)m_IntakeSystem.Unstow();
    else if(!flag1) m_IntakeSystem.reverseIntake();

    if(!m_IntakeSystem.HasCube() && !flag1) {
      time1=Timer.getFPGATimestamp();
      flag1=true;
    }
      
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
//    m_ArmSubsystem.StopMotors();
    m_ArmSubsystem.setMagicSlow();
    m_IntakeSystem.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

