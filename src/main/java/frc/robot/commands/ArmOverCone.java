package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;

public class ArmOverCone extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
 
    private ArmSubsystem arm;
    private IntakeSystem intake;
    private JoystickButton btn;
    double time1=0;
    boolean  flag1=false,flag2=false;
  public ArmOverCone(ArmSubsystem m_arm,IntakeSystem m_intake,JoystickButton m_btn) {
    arm=m_arm;
    intake = m_intake;
    btn=m_btn;
    addRequirements(arm,intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setMagicFast();
    arm.motorActive=true;
    arm.setPositionInCounts(arm.posOverCone);
  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    double counts = arm.getCounts();
    
    if(counts < 1250 && !flag1)intake.Unstow();
    if(counts>=1250) intake.reverseIntake();
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
//    m_ArmSubsystem.StopMotors();
    arm.setMagicSlow();
    intake.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !btn.getAsBoolean();
  }
}

