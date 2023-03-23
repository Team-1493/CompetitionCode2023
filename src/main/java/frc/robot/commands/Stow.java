package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;

public class Stow extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
 
    private ArmSubsystem m_ArmSubsystem;
    private IntakeSystem m_IntakeSystem;
    private double lsTime;
    private double currentTime;
    private double timeToRunWheels=0.5;

  public Stow(ArmSubsystem arm,IntakeSystem intake) {
    m_ArmSubsystem = arm;
    m_IntakeSystem = intake;

    addRequirements(arm,intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmSubsystem.resetIntegralAccumulator();  
    m_IntakeSystem.stowRollers();
    currentTime=Timer.getFPGATimestamp();
    lsTime=currentTime;

  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    currentTime=Timer.getFPGATimestamp();

    if(m_ArmSubsystem.getLowerLimitSwitch() ) {     
//        m_ArmSubsystem.setPositionInCounts(m_ArmSubsystem.posStow);
          m_ArmSubsystem.setArmPercentOutput(-.08);
        lsTime=currentTime;
    }

    else{     
      m_ArmSubsystem.StopMotors();    
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.StopMotors();
    m_IntakeSystem.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//    return !m_ArmSubsystem.getLowerLimitSwitch();

//    checks to see if sufficient time has passed since triggering limit switch 
    return ( currentTime-lsTime>=timeToRunWheels)|| m_ArmSubsystem.getCounts()<1150;
  }
}

