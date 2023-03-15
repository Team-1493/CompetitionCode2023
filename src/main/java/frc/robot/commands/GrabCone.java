package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;

public class GrabCone extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private IntakeSystem m_IntakeSystem;
    private ArmSubsystem m_arm;
    private JoystickButton m_btn;


  public GrabCone(IntakeSystem intake, ArmSubsystem arm, JoystickButton btn) {
    m_IntakeSystem = intake;
    m_arm=arm;
    m_btn=btn;
    addRequirements(intake,arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSystem.GrabCone();
    m_arm.motorActive=true;
    m_arm.setPositionInCounts(m_arm.posConeGrab);
  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.StopMotors();
    m_arm.setPositionInCounts(m_arm.posOverCone);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!m_btn.getAsBoolean())||(m_IntakeSystem.getRearIntakeBarCurrent()>15)||
    (m_IntakeSystem.getFrontIntakeBarCurrent()>15);
  }
}

