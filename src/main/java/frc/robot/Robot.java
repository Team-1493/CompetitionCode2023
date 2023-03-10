// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  private static final String kTestAuto2 = "Test Auto 2";
  private static final String kTestAuto3 = "Test Auto 3";
  private static final String kAuto1 = "Auto 1";
  private String m_autoSelected;
  public static boolean enabled;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  Command testAuto1Command, testAuto2Command, testAuto3Command, auto1Command;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings
    m_robotContainer = new RobotContainer();
    testAuto1Command=m_robotContainer.getTestAutonomousCommand1();
    testAuto2Command=m_robotContainer.getTestAutonomousCommand2();
    testAuto3Command=m_robotContainer.getTestAutonomousCommand3();
    auto1Command=m_robotContainer.getAutonomousCommand1();
    m_chooser.setDefaultOption("Auto 1", kAuto1);
    m_chooser.addOption("Test Auto 1", kTestAuto1);
    m_chooser.addOption("Test Auto 2", kTestAuto2);
    m_chooser.addOption("Test Auto 3", kTestAuto3);
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

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    enabled=true;
    m_robotContainer.setPIDslot(1);  // use the auto PID gains for auto

    m_autoSelected = m_chooser.getSelected();

    switch (m_autoSelected) {
      case kAuto1:
        m_autonomousCommand=auto1Command;
        break;
      case kTestAuto1:
        m_autonomousCommand=testAuto1Command;
        break;
      case kTestAuto2:
        m_autonomousCommand=testAuto2Command;
        break;
      case kTestAuto3:
        m_autonomousCommand=testAuto3Command;
      default:
        m_autonomousCommand=auto1Command;
        break;
    }
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {        
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    enabled=true;
     if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setPIDslot(0);  // use the auto PID gains for teleop
//  need to call re-enable gyro?  See 2022 code
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }


}
