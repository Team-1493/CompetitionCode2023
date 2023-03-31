// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.AutoGenerator;
import frc.robot.commands.DriveStick;
import frc.robot.commands.DropCone;
import frc.robot.commands.FollowLimelight;
import frc.robot.commands.GrabCone;
import frc.robot.commands.IntakeFromShooter;
import frc.robot.commands.ReachForCone;
import frc.robot.commands.SpitCube;
import frc.robot.commands.CubeIntake;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ShootCube;
import frc.robot.commands.Stow;
import frc.robot.commands.autobalancer2;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.LEDsystem;
import frc.robot.subsystems.Stick;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.PreAutoBalancer;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here..
  public final Stick driverJoystick =new Stick(0);
  public final Stick operatorJoystick =new Stick(1);

  public JoystickButton btnRot0 = driverJoystick.getButton(1);
  public JoystickButton btnRot90 = driverJoystick.getButton(2);
  public JoystickButton btnRotneg90 = driverJoystick.getButton(3);
  public JoystickButton btnRot180 = driverJoystick.getButton(4);
  public JoystickButton btnAlignAprilTag =  driverJoystick.getButton(4);

  // button 5 used for slow rotate in Stick
  public JoystickButton btnResetGyro = driverJoystick.getButton(6);
  public JoystickButton btnAprilTagAlign = driverJoystick.getButton(7);
  public JoystickButton btnUpdateConstants = driverJoystick.getButton(8);
    
  public JoystickButton btnShootCube2 = operatorJoystick.getButton(1); //B - med\
  public JoystickButton btnShootCube1 = operatorJoystick.getButton(2); // A - low
  public JoystickButton btnShootCube3 = operatorJoystick.getButton(3); // Y - high
  public JoystickButton btnShootCubeFarLow = operatorJoystick.getButton(4); //X - far floor
  public JoystickButton btnIntakeCube = operatorJoystick.getButton(6);
  public JoystickButton btnReverseIntake = operatorJoystick.getButton(7);
  public JoystickButton btnShootCubeFarHigh = operatorJoystick.getButton(8); // 8 - far high
  public JoystickButton btnReachForCone = operatorJoystick.getButton(9);
  public JoystickButton btnStow = operatorJoystick.getButton(5);

  
  


  private final ArmSubsystem m_ArmSystem = new ArmSubsystem();
  public final SwerveDrive m_swervedriveSystem = new SwerveDrive();
  // public final Limelight m_Limelight = new Limelight();
  public final IntakeSystem m_IntakeSystem = new IntakeSystem();
  public final AutoGenerator autoGenerator = new AutoGenerator(m_swervedriveSystem, m_ArmSystem, m_IntakeSystem);
  public final LEDsystem led = new LEDsystem();
  public final PreAutoBalancer m_preautobalancer = new PreAutoBalancer(m_swervedriveSystem);
  // public final 


  Supplier<double[]> driverStickState = () -> driverJoystick.readStick();
  Supplier<double[]> operatorStickState = () -> operatorJoystick.readStick();

  public final Stow stowCommand = new Stow(m_ArmSystem,m_IntakeSystem);
  public final Stow stowCommand2 = new Stow(m_ArmSystem,m_IntakeSystem);
  public final ReverseIntake reverseIntake = new ReverseIntake(m_ArmSystem,m_IntakeSystem);

  public final CubeIntake cubeIntake = new CubeIntake(m_ArmSystem,m_IntakeSystem,btnIntakeCube);
  public final SpitCube SpitCubeCommand = 
      new SpitCube(m_ArmSystem, m_IntakeSystem);
  
  public final ReachForCone reachForCone = new ReachForCone(m_ArmSystem,m_IntakeSystem, btnReverseIntake);
  public final GrabCone grabCone = new GrabCone(m_IntakeSystem,m_ArmSystem, btnReverseIntake);


  public final DriveStick driveCommand = new DriveStick(m_swervedriveSystem,driverStickState); 

//  public JoystickButton btnAimAtTape = driverJoystick.getButton(1);
//public JoystickButton btnFollowLimelight = driverJoystick.getButton(5);




  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swervedriveSystem.setDefaultCommand(driveCommand);

    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {

// driver functions    
    new Trigger(btnResetGyro).onTrue(new ResetGyro(m_swervedriveSystem));
//    new Trigger(btnUpdateConstants).onTrue( new InstantCommand(()-> updateConstants()));   

/* 
    var thingy = new SequentialCommandGroup(
    (Command)m_preautobalancer,
      (Command)m_autobalancer
  );
  
new Trigger(driverJoystick.getButton(7)).whileTrue
         (thingy);
*/

    new Trigger(btnRot0).onTrue
        (new InstantCommand( ()-> m_swervedriveSystem.resetRotatePID(0)));
    new Trigger(btnRot90).onTrue
       (new InstantCommand( ()-> m_swervedriveSystem.resetRotatePID(Math.PI/2)));
    new Trigger(btnRot180).onTrue
       (new InstantCommand( ()-> m_swervedriveSystem.resetRotatePID(Math.PI)));
    new Trigger(btnRotneg90).onTrue
       (new InstantCommand( ()-> m_swervedriveSystem.resetRotatePID(-Math.PI/2)));

    // new Trigger(btnAprilTagAlign).whileTrue(new FollowLimelight(m_swervedriveSystem,m_Limelight));

// operator functions
    new Trigger(btnShootCube1).whileTrue(new ShootCube(m_IntakeSystem,1));
    new Trigger(btnShootCube2).whileTrue(new ShootCube(m_IntakeSystem,2));
    new Trigger(btnShootCube3).whileTrue(new ShootCube(m_IntakeSystem,3));
    new Trigger(btnShootCubeFarLow).whileTrue(new ShootCube(m_IntakeSystem,4));
    new Trigger(btnShootCubeFarHigh).whileTrue(new ShootCube(m_IntakeSystem,5));
    new Trigger(operatorJoystick.mapStick(3)).whileTrue
      (new IntakeFromShooter(m_IntakeSystem));
    
      new Trigger(btnIntakeCube).onTrue(new SequentialCommandGroup(cubeIntake,stowCommand));
   
    
    new Trigger(btnReverseIntake).whileTrue(SpitCubeCommand);
    new Trigger(btnReverseIntake).onFalse(stowCommand2);

    new Trigger(btnReachForCone).whileTrue(reachForCone);
    new Trigger(btnStow).whileTrue(stowCommand2);

  }


  public Command getTestAuto1() {
    // An example command will be run in autonomous
    return autoGenerator.testAutoCommand1();
  }

  public Command getAutonomousBalanceFromMiddle(){
    return autoGenerator.autoBalanceFromMiddle();
  }

  public Command getAutonomousShootHigh(){
    return autoGenerator.autoShootHigh();
  }



//*************   
  public Command getAutonomousRedLeft2(){
    return autoGenerator.autoRedLeft2();
  }
  public Command getAutonomousBlueLeft2(){
    return autoGenerator.autoBlueLeft2();
  }



//*************   
  public Command getAutonomousRedLeft2Bal(){
    return autoGenerator.autoRedLeft2Bal();
  }
  public Command getAutonomousBlueLeft2Bal(){
    return autoGenerator.autoBlueLeft2Bal();
  }


//*************   
  public Command getAutonomousRedLeftBalance1(){
    return autoGenerator.autoRedLeftBal1();
  }
  public Command getAutonomousBlueLeftBalance1(){
    return autoGenerator.autoBlueLeftBal1();
  }


//*************   
public Command getAutonomousRedLeftReturn2(){
  return autoGenerator.autoRedLeftReturn2();
}
public Command getAutonomousBlueLeftReturn2(){
  return autoGenerator.autoBlueLeftReturn2();
}


// ****************************
public Command getAutonomousRedRightBal3(){
  return autoGenerator.autoRedRightBal3();
}
public Command getAutonomousBlueRightBal3(){
  return autoGenerator.autoBlueRightBal3();
}


//***************************
public Command getAutonomousRedRight3(){
  return autoGenerator.autoRedRight3();
}
public Command getAutonomousBlueRight3(){
  return autoGenerator.autoBlueRight3();
}


//***************************
  public Command getAutonomousRedRightBal2(){
    return autoGenerator.autoRedRightBal2();
  }
  public Command getAutonomousBlueRightBal2(){
    return autoGenerator.autoBlueRightBal2();
  }







  

// We have different PID constants for the drive wheels between teleop and auto
// Switch between slot 0 for teleop and slot 1 for auto 
  public void setPIDslot(int slot){
    m_swervedriveSystem.setPIDSlot(slot);
  }

  public void turnOnRamp(){
    m_swervedriveSystem.turnOnRamp();
  }

  public void turnOffRamp(){
    m_swervedriveSystem.turnOffRamp();
  }

  public void turnOnVoltageComp(){
    m_swervedriveSystem.turnOnVoltageComp();;
  }

  public void turnOffVoltageComp(){
    m_swervedriveSystem.turnOffVoltageComp();
  }

  public void updateConstants(){
    m_swervedriveSystem.updateConstants();
    m_ArmSystem.updateConstants();
    m_IntakeSystem.UpdateConstants(); 
  }


}
