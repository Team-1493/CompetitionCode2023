package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.SwerveDrive;

public class ResetOdometryAuto  extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private SwerveDrive sds;
    private Pose2d pose;
    private double start_time;
    
  public ResetOdometryAuto(SwerveDrive m_sds, Pose2d m_pose) {
    sds=m_sds;
    pose=m_pose;
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start_time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    sds.resetOdometry(pose);
    System.out.println("IIIIIIIIIIIIIIIIIIIIIIIIII"+pose.getX()+"   "+pose.getY());

  } 

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("FFFFFFFFFFFFFFFFFFFFF"+sds.getPose().getX()+"   "+sds.getPose().getY());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp()-start_time>.125;
  }
}

