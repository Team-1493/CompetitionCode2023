package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;

public class CubeIntakeAuto extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private ArmSubsystem arm;
    private IntakeSystem intake;

  public CubeIntakeAuto(ArmSubsystem m_arm,IntakeSystem m_intake) {
    arm = m_arm;
    intake = m_intake;
    addRequirements(arm,intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.resetIntegralAccumulator();
    intake.Unstow();
    arm.setArmPercentOutput(0.5);
    arm.motorActive=true;

  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    if (arm.getCounts() >1250 || !arm.getLowerLimitSwitch())  arm.setPositionInCounts(arm.posCubeIntake);
    intake.IntakeCube();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    boolean ir = intake.HasCube();
    return ir;
  }
}

