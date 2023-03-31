package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
    public int armfaceFwd = 0;
    double scaleFactor=1/60.; //was 32
    double xposStart;
    double pitch,pitchPrev,deltaPitch;
    double timeBalance;

  

  public autobalancer2(SwerveDrive m_sds,double m_sign) {
    sds=m_sds;
    directionRad=0;
    // sign = 1 for arm first, sign = -1 for shooter first
    sign=m_sign;
    addRequirements(sds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
//    sds.resetGyro();
    pitch=sds.pitch;
    pitchPrev=pitch;
    deltaPitch=0;

    timeBalance=Timer.getFPGATimestamp();
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitch=sds.pitch;
    deltaPitch=pitch-pitchPrev;
    double vx=-pitch*scaleFactor*sign;
    if(Math.abs(pitch)<5) vx=0;
//    if(pitch>0 && deltaPitch<1.5) vx=0;
//    if(pitch<0 && deltaPitch>1.5) vx=0;
//    System.out.println(pitch+"   "+pitchPrev+"   "+deltaPitch+"   "+vx);

    pitchPrev=pitch;
    sds.setMotors(vx, 0, 0);
    
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sds.allStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
//      return Math.abs(pitch)<5;

    if(Math.abs(pitch)<5)timeBalance=Timer.getFPGATimestamp();
    return (Timer.getFPGATimestamp()-timeBalance>2);
  }
}

