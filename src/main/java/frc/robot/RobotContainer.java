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
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.ArmOverCone;
import frc.robot.commands.CubeIntake;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.RotateInPlace;
import frc.robot.commands.ShootCube;
import frc.robot.commands.Stow;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.Stick;
import frc.robot.subsystems.Limelight;

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
  private final ArmSubsystem m_ArmSystem = new ArmSubsystem();
  public final SwerveDrive m_swervedriveSystem = new SwerveDrive();
  public final Limelight m_LimelightBack = new Limelight();
  public final IntakeSystem m_IntakeSystem = new IntakeSystem();
  public final AutoGenerator autoGenerator = new AutoGenerator(m_swervedriveSystem, m_ArmSystem, m_IntakeSystem);
  public final Stick driverJoystick =new Stick(0);
  public final Stick operatorJoystick =new Stick(1);

  Supplier<double[]> driverStickState = () -> driverJoystick.readStick();
  Supplier<double[]> operatorStickState = () -> operatorJoystick.readStick();

  public final Stow stowCommand = new Stow(m_ArmSystem,m_IntakeSystem);
  public final ReverseIntake reverseIntake = new ReverseIntake(m_ArmSystem,m_IntakeSystem);
  public JoystickButton btnIntakeCube = operatorJoystick.getButton(6);

  public final CubeIntake cubeIntake = new CubeIntake(m_ArmSystem,m_IntakeSystem,btnIntakeCube);
  public final ArmOverCone armOverConeCommand = 
      new ArmOverCone(m_ArmSystem, m_IntakeSystem);
  public final ArmDefaultCommand armDefault=
      new ArmDefaultCommand(m_ArmSystem,operatorStickState);

  public final DriveStick driveCommand = new DriveStick(m_swervedriveSystem,driverStickState); 

  public JoystickButton btnAimAtTape = driverJoystick.getButton(1);
  public JoystickButton btnResetGyro = driverJoystick.getButton(6);
  public JoystickButton btnUpdateConstants = driverJoystick.getButton(3);
  public JoystickButton btnFollowLimelight = driverJoystick.getButton(4);
  public JoystickButton btnDropCone = driverJoystick.getButton(5); //L1
//  public JoystickButton btnGrabCone = driverJoystick.getButton(6); // R1


  public JoystickButton btnShootCube1 = operatorJoystick.getButton(2); // A
  public JoystickButton btnShootCube2 = operatorJoystick.getButton(1); //B
  public JoystickButton btnShootCube3 = operatorJoystick.getButton(3); // Y
  public JoystickButton btnShootCubeFarLow = operatorJoystick.getButton(4); //X
  public JoystickButton btnShootCubeFarHigh = operatorJoystick.getButton(8);
  
  public POVButton btnShootCubeAuto = operatorJoystick.pov180;


 // public JoystickButton btnReverseIntake = operatorJoystick.getButton(8);
  public JoystickButton btnStow = operatorJoystick.getButton(9);
 
  public JoystickButton btnArmToGetcone = operatorJoystick.getButton(10);



  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swervedriveSystem.setDefaultCommand(driveCommand);
    m_ArmSystem.setDefaultCommand(armDefault);

    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {

// driver functions    
    new Trigger(btnResetGyro).onTrue(new ResetGyro(m_swervedriveSystem));
    new Trigger(btnUpdateConstants).onTrue( new InstantCommand(()-> updateConstants()));   

    new Trigger(driverJoystick.pov0).onTrue
       (new InstantCommand( ()-> m_swervedriveSystem.resetRotatePID(0)));
    new Trigger(driverJoystick.pov90).onTrue
       (new InstantCommand( ()-> m_swervedriveSystem.resetRotatePID(Math.PI/2)));
    new Trigger(driverJoystick.pov180).onTrue
       (new InstantCommand( ()-> m_swervedriveSystem.resetRotatePID(Math.PI)));
    new Trigger(driverJoystick.pov270).onTrue
       (new InstantCommand( ()-> m_swervedriveSystem.resetRotatePID(-Math.PI/2)));

/*     
    new Trigger(driverJoystick.pov0).onTrue(new RotateInPlace(m_swervedriveSystem, 0));
    new Trigger(driverJoystick.pov90).onTrue(new RotateInPlace(m_swervedriveSystem, Math.PI/2));
    new Trigger(driverJoystick.pov180).onTrue(new RotateInPlace(m_swervedriveSystem, Math.PI));  
    new Trigger(driverJoystick.pov270).onTrue(new RotateInPlace(m_swervedriveSystem, -Math.PI/2));
 */

//    new Trigger(btnGrabCone).whileTrue(new GrabCone(m_IntakeSystem,m_ArmSystem));
//    new Trigger(btnDropCone).whileTrue(new DropCone(m_IntakeSystem,m_ArmSystem));


// operator functions

    new Trigger(btnShootCube1).whileTrue(new ShootCube(m_IntakeSystem,1));
    new Trigger(btnShootCube2).whileTrue(new ShootCube(m_IntakeSystem,2));
    new Trigger(btnShootCube3).whileTrue(new ShootCube(m_IntakeSystem,3));
    new Trigger(btnShootCubeFarLow).whileTrue(new ShootCube(m_IntakeSystem,4));
    new Trigger(btnShootCubeFarHigh).whileTrue(new ShootCube(m_IntakeSystem,5));

    new Trigger(operatorJoystick.mapStick(3)).whileTrue(new IntakeFromShooter(m_IntakeSystem));

    
//    new Trigger(btnIntakeCube).whileTrue(cubeIntake);
//    new Trigger(btnIntakeCube).onFalse(stowCommand);
      new Trigger(btnIntakeCube).onTrue(new SequentialCommandGroup(cubeIntake,stowCommand));

//    new Trigger(btnReverseIntake).whileTrue(armOverConeCommand.andThen(reverseIntake));
//    new Trigger(btnReverseIntake).onFalse(stowCommand);

    new Trigger(btnStow).onTrue(stowCommand);

  }


  public Command getTestAutonomousCommand1() {
    // An example command will be run in autonomous
    return autoGenerator.testAutoCommand1();
  }

  public Command getTestAutonomousCommand2() {
    // An example command will be run in autonomous
    return autoGenerator.testAutoCommand2();
  }

  public Command getTestAutonomousCommand3() {
    //An example command will be run in autonomous
    return autoGenerator.testAutoCommand3();
  }

  public Command getAutonomousCommand1(){
    autoGenerator.updatePID();
    return autoGenerator.autoCommand1();
  }


// We have different PID constants for the drive wheels between teleop and auto
// Switch between slot 0 for teleop and slot 1 for auto 
  public void setPIDslot(int slot){
    m_swervedriveSystem.setPIDSlot(slot);
  }

  public void updateConstants(){
    m_swervedriveSystem.updateConstants();
    m_ArmSystem.updateConstants();
    m_IntakeSystem.UpdateConstants(); 
  }


}
