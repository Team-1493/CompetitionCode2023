// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private static final String kTestAuto1 = "Test Auto 1";

  private static final String kRRB = "RedRightBal";
  private static final String kRLB = "RedLeftBal";
  private static final String kRR = "RedRight";
  private static final String kRL = "RedLeft";
  
  private static final String kBLB = "BlueLeftBal";
  private static final String kBRB = "BlueRightBal";
  private static final String kBL = "BlueLeft";
  private static final String kBR = "BlueRight";
  private static final String kBLS = "pathBlueLeftStay";
  private static final String kBRS = "pathBlueRightStay";

  private static final String kMidBal = "Middle Balance";
  private static final String kAutoShootHigh = "Auto Shoot High";

  private String m_autoSelected;
  public static boolean enabled;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  static public int inAuto=0;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings
    m_robotContainer = new RobotContainer();
//    PathPlannerServer.startServer(5811);
    
    m_chooser.setDefaultOption("Red Right Bal", kRRB);
    m_chooser.addOption("Red Left Bal", kRLB);
    m_chooser.addOption("Red Right", kRR);
    m_chooser.addOption("Red Left", kRL);

    m_chooser.addOption("Blue Left Bal", kBLB);
    m_chooser.addOption("Blue Right Bal", kBRB);
    m_chooser.addOption("Blue Left ", kBL);
    m_chooser.addOption("Blue Right ", kBR);
    m_chooser.addOption("Middle Balance ", kMidBal);
//    m_chooser.addOption("Blue Left Two Cubes Return", kBLS);
//    m_chooser.addOption("Blue Right Two Cubes Return", kBRS);
    
    m_chooser.addOption("Shoot High", kAutoShootHigh);
   
    m_chooser.addOption("Test Auto 1", kTestAuto1);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {enabled=false;}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selercted by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    inAuto=1;
    m_robotContainer.turnOffRamp();
    m_robotContainer.turnOnVoltageComp();
    enabled=true;
    m_robotContainer.setPIDslot(1);  // use the auto PID gains for auto

    m_autoSelected = m_chooser.getSelected();

    switch (m_autoSelected) {
      case kRRB:
        m_autonomousCommand=m_robotContainer.getAutonomousCommandRedRightBal();
        break;
      
      case kRLB:
        m_autonomousCommand=m_robotContainer.getAutonomousCommandRedLeftBal();
      break;

      case kRR:
        m_autonomousCommand=m_robotContainer.getAutonomousCommandRedRight();;
      break;

      case kRL:
      m_autonomousCommand=m_robotContainer.getAutonomousCommandRedLeft();;
      break;

      case kBLB:
        m_autonomousCommand=m_robotContainer.getAutonomousCommandBlueLeftBal();;
        break;
     
      case kBRB:
        m_autonomousCommand=m_robotContainer.getAutonomousCommandBlueRightBal();
        break;

      case kBL:
        m_autonomousCommand=m_robotContainer.getAutonomousCommandBlueLeft();
      break;   

      case kBR:
        m_autonomousCommand=m_robotContainer.getAutonomousCommandBlueRight();
      break; 

      case kMidBal:
      m_autonomousCommand=m_robotContainer.getAutonomousBalanceFromMiddle();
    break; 

//      case kAutoBLStay:
//        m_autonomousCommand=m_robotContainer.getAutonomousCommandBlueLeftStay();
//        break;
//      case kAutoBRStay:
//        m_autonomousCommand=m_robotContainer.getAutonomousCommandBlueRightStay();
//        break;
      case kAutoShootHigh:
        m_autonomousCommand= m_robotContainer.getAutonomousShootHigh();;
        break;                
      case kTestAuto1:
        m_autonomousCommand=m_robotContainer.getTestAuto1();
        break;

      default:
        m_autonomousCommand=m_robotContainer.getAutonomousCommandRedRightBal();
        break;
    }
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {        
      System.out.println("***************************"+m_autoSelected);
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_robotContainer.m_swervedriveSystem.allStop();
    }
    inAuto=0;
    m_robotContainer.turnOnRamp();
    m_robotContainer.turnOffVoltageComp();
    enabled=true;
    m_robotContainer.setPIDslot(0);  // use the auto PID gains for teleop
//  need to call re-enable gyro?  See 2022 code
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }


}
