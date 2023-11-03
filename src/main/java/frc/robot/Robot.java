// Copy` (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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

  private String m_autoSelected;
  public static boolean enabled;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  static public int inAuto=0;
  private Command autoBalanceFromMiddle;
  private Command autoShootHigh;
  private Command autoRedLeft2,autoBlueLeft2;
  private Command autoRedLeft2Bal,autoBlueLeft2Bal;
  private Command autoRedLeftBalance1,autoBlueLeftBalance1;
  private Command autoRedLeftReturn2,autoBlueLeftReturn2;
  private Command autoRedRightBal3,autoBlueRightBal3;
  private Command autoRedRight3,autoBlueRight3;
  private Command autoRedRightBal2,autoBlueRightBal2;
  private Command autoRedWallExit, autoRedBumpExit, autoBlueWallExit, autoBlueBumpExit;   
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Timer.delay(3);
    // Instantiate our RobotContainer.  This will perform all our button bindings
    m_robotContainer = new RobotContainer();
//    PathPlannerServer.startServer(5811);
    
    m_chooser.setDefaultOption("Test Auto 1", "Test Auto 1");
    m_chooser.addOption("Shoot High", "Shoot High");
    m_chooser.addOption("Middle Balance", "Middle Balance");
    m_chooser.addOption("Test Auto 1", "Test Auto 1");

    m_chooser.addOption("Red Left 2", "Red Left 2");
    m_chooser.addOption("Blue Left 2", "Blue Left 2");
    
    m_chooser.addOption("Red Left 2 Balance", "Red Left 2 Balance");
    m_chooser.addOption("Blue Left 2 Balance", "Blue Left 2 Balance");
    
    m_chooser.addOption("Red Left Balance 1", "Red Left Balance 1");
    m_chooser.addOption("Blue Left Balance 1", "Blue Left Balance 1");

    m_chooser.addOption("Red Left Return 2", "Red Left Return 2");    
    m_chooser.addOption("Blue Left Return 2", "Blue Left Return 2");    


    m_chooser.addOption("Red Right Balance 3", "Red Right Balance 3");
    m_chooser.addOption("Blue Right Balance 3", "Blue Right Balance 3");


    m_chooser.addOption("Red Right 3", "Red Right 3");
    m_chooser.addOption("Blue Right 3", "Blue Right 3");


    m_chooser.addOption("Red Right Balance 2", "Red Right Balance 2");
    m_chooser.addOption("Blue Right Balance 2", "Blue Right Balance 2");


    m_chooser.addOption("Red Wall Exit", "Red Wall Exit");
    m_chooser.addOption("Red Bump Exit", "Red Bump Exit");
    m_chooser.addOption("Blue Wall Exit", "Blue Wall Exit");
    m_chooser.addOption("Blue Bump Exit", "Blue Bump Exit");

    SmartDashboard.putData("Auto choices", m_chooser);


    autoBalanceFromMiddle = m_robotContainer.getAutonomousBalanceFromMiddle();
    
    autoShootHigh = m_robotContainer.getAutonomousShootHigh();

    autoRedLeft2 = m_robotContainer.getAutonomousRedLeft2();
    autoBlueLeft2 = m_robotContainer.getAutonomousBlueLeft2();
    
    autoRedLeft2Bal = m_robotContainer.getAutonomousRedLeft2Bal();
    autoBlueLeft2Bal = m_robotContainer.getAutonomousBlueLeft2Bal();
    
    autoRedLeftBalance1 = m_robotContainer.getAutonomousRedLeftBalance1();
    autoBlueLeftBalance1 = m_robotContainer.getAutonomousBlueLeftBalance1();

    autoRedLeftReturn2 = m_robotContainer.getAutonomousRedLeftReturn2();
    autoBlueLeftReturn2 = m_robotContainer.getAutonomousBlueLeftReturn2();

    autoRedRightBal3 = m_robotContainer.getAutonomousRedRightBal3();
    autoBlueRightBal3 = m_robotContainer.getAutonomousBlueRightBal3();

    autoRedRight3 = m_robotContainer.getAutonomousRedRight3();
    autoBlueRight3 = m_robotContainer.getAutonomousBlueRight3();

    autoRedRightBal2 = m_robotContainer.getAutonomousRedRightBal2();
    autoBlueRightBal2 = m_robotContainer.getAutonomousBlueRightBal2();
    
    autoRedWallExit = m_robotContainer.getAutonomousRedWallExit();
    autoRedBumpExit = m_robotContainer.getAutonomousRedBumpExit();
    autoBlueWallExit = m_robotContainer.getAutonomousBlueWallExit();
    autoBlueBumpExit = m_robotContainer.getAutonomousBlueBumpExit();


    


// set up for auto    
    inAuto=1;
    m_robotContainer.turnOffRamp();
    m_robotContainer.turnOnVoltageComp();
    m_robotContainer.setPIDslot(1);  // use the auto PID gains for auto



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
    if (inAuto !=1){
      inAuto=1;
      m_robotContainer.turnOffRamp();
      m_robotContainer.turnOnVoltageComp();
      m_robotContainer.setPIDslot(1);  // use the auto PID gains for auto
      enabled=true;
    }   

    m_autoSelected = m_chooser.getSelected();

    switch (m_autoSelected) {
    
    case "Test Auto 1":
        m_autonomousCommand=m_robotContainer.getTestAuto1();
    break;

    case "Middle Balance":
      m_autonomousCommand=autoBalanceFromMiddle;
    break; 

      case "Shoot High":
        m_autonomousCommand= autoShootHigh;
      break;                


      case "Red Left 2":
        m_autonomousCommand=autoRedLeft2;
      break;
      case "Blue Left 2":
        m_autonomousCommand=autoBlueLeft2;
      break;


      case "Red Left 2 Balance":
        m_autonomousCommand=autoRedLeft2Bal;
      break;
      case "Blue Left 2 Balance":
        m_autonomousCommand=autoBlueLeft2Bal;
      break;


    case "Red Left Balance 1":
        m_autonomousCommand=autoRedLeftBalance1;
    break;
    case "Blue Left Balance 1":
        m_autonomousCommand=autoBlueLeftBalance1;
    break;


    case "Red Left Return 2":
      m_autonomousCommand=autoRedLeftReturn2;
    break;
    case "Blue Left Return 2":
      m_autonomousCommand=autoBlueLeftReturn2;
    break;


    case "Red Right Balance 3":
      m_autonomousCommand=autoRedRightBal3;
    break;
    case "Blue Right Balance 3":
      m_autonomousCommand=autoBlueRightBal3;
    break;


    case "Red Right 3":
      m_autonomousCommand=autoRedRight3;
    break;
    case "Blue Right 3":
      m_autonomousCommand=autoBlueRight3;
    break;


    case "Red Right Balance 2":
      m_autonomousCommand=autoRedRightBal2;
    break;
    case "Blue Right Balance 2":
      m_autonomousCommand=autoBlueRightBal2;
    break;

    case "Red Wall Exit":
      m_autonomousCommand=autoRedWallExit;
    break;
  
    case "Red Bump Exit":
      m_autonomousCommand=autoRedBumpExit;
    break;

    case "Blue Wall Exit":
      m_autonomousCommand=autoBlueWallExit;
    break;
  
    case "Blue Bump Exit":
      m_autonomousCommand=autoBlueBumpExit;
    break;

  

    default:
      m_autonomousCommand=autoShootHigh;
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
    if(inAuto !=0) {
      inAuto=0;
      m_robotContainer.turnOnRamp();
      m_robotContainer.turnOffVoltageComp();
      m_robotContainer.setPIDslot(0);  // use the auto PID gains for teleop
      enabled=true;
    }

//  need to call re-enable gyro?  See 2022 code
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }


}
