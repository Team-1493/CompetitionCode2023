package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.SwerveDrive;

public class RotateInPlace extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private SwerveDrive sd;
    private double goal;

  public RotateInPlace(SwerveDrive m_sd,double m_goal) {
        sd=m_sd;
        goal=m_goal;
    addRequirements(sd);
  }

  // Called when the command is initially scheduled.
  // resets pidController to current heading and sets the desired rotation goal
  @Override
  public void initialize() {
    sd.rotatePIDon=true;
    sd.resetRotatePID(goal);
    

  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    sd.setMotors(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sd.rotatePIDon=false;
    sd.allStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(sd.heading-goal)<0.01;
  }
}

