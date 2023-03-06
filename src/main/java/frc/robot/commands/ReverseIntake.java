package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;

public class ReverseIntake extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private ArmSubsystem m_ArmSubsystem;
    private IntakeSystem m_IntakeSystem;

  public ReverseIntake(ArmSubsystem arm,IntakeSystem intake) {
    m_ArmSubsystem = arm;
    m_IntakeSystem = intake;
    addRequirements(arm,intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmSubsystem.resetIntegralAccumulator();
    m_IntakeSystem.reverseIntake();
    m_ArmSubsystem.motorActive=true;

  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    m_ArmSubsystem.setPositionInCounts(m_ArmSubsystem.posCubeIntake);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IntakeSystem.HasCube();
  }
}

