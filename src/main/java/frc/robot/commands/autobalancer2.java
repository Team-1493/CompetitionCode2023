package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.PreAutoBalancer;

public class autobalancer2 extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private SwerveDrive sds;
    private double direction,directionRad;
    private double sign;
    double scaleFactor=1./20.;
    double xposStart;
  

  public autobalancer2(SwerveDrive m_sds) {
    sds=m_sds;
    direction=0;
    directionRad=direction*Math.PI/180;
    addRequirements(sds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sign=-1;
    if (sds.heading>90 || sds.heading <-90) sign=1;
    
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     double pitch=sds.pitch;
    double vx=pitch*scaleFactor;
    double heading = sds.heading;
    double omega=(heading-directionRad)*2;
    if(Math.abs(pitch)<2) vx=0;
    if(Math.abs(omega)<0.02) omega=0;
    System.out.println("pitch = "+pitch+"    vx = "+
      vx+"   heading = "+heading+"     omega = "+omega);
    sds.setMotors(vx*sign, 0, 0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sds.allStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return false;
  }
}

