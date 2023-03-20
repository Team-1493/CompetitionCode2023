// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSystem extends SubsystemBase {
    final TalonFX TopConveyor;
    final TalonFX FrontIntakeBar;
    final TalonFX RearIntakeBars;
    final TalonFX ShooterTop;
    final TalonFX ShooterBottom;
    final DigitalInput irSensorL;
    
    private double topConveyorIntakeSpeed = 700;
    private double topConveyorRevIntakeSpeed = 700; 
    private double topConveyorInjectSpeed = 0.35;


    private double ShootSpeedUpper1=325;
    private double ShootSpeedLower1=100;

    private double ShootSpeedUpper2=650;
    private double ShootSpeedLower2=650;

    private double ShootSpeedUpper3=905;
    private double ShootSpeedLower3=905;

    private double ShootSpeedUpper4=1325;
    private double ShootSpeedLower4=1125;

    private double ShootSpeedUpper5=2000;
    private double ShootSpeedLower5=1975;

    private double ShooterUpperRevSetSpeed=-1000;
    private double ShooterLowerRevSetSpeed=-1000;

    private double cubeIntakePower = 0.20;
    private double frontIntakePower = 0.2;
    private double rearIntakePower = 0.2;
    private double stowRollerSpeed = -0.4;
    private double unstowRollerSpeed =1.0;

    private double topConveyorKf = .244;
    private double topConveyorKp = 0;
    private double topConveyorKi = 0;
    private double topConveyorKizone = 0;


    private double shooterKf = .244;
    private double shooterKp = -0.012;  // NOT SURE WHY NEGATIVE IS BEST?
    private double shooterKi = 0.0;
    private double shooterKizone = 0.0;

    private double ShooterUpperSetSpeed=0;
    private double ShooterLowerSetSpeed=0;
    private int shootLevel;
    static boolean sensorValue=true;

    
    public IntakeSystem() {
        SmartDashboard.putNumber("Top Conveyor Intake Speed",topConveyorIntakeSpeed);
        SmartDashboard.putNumber("Top Conveyor Inject Speed",topConveyorInjectSpeed);
        SmartDashboard.putNumber("Top Conveyor Intake Rev Speed",topConveyorRevIntakeSpeed);
        SmartDashboard.putNumber("Front Intake Power",frontIntakePower);
        SmartDashboard.putNumber("Rear Intake Power",rearIntakePower);
        SmartDashboard.putNumber("Stow Roller Speed",stowRollerSpeed);
        SmartDashboard.putNumber("Unstow Roller Speed",unstowRollerSpeed);


        SmartDashboard.putNumber("Shoot Speed U1", ShootSpeedUpper1);
        SmartDashboard.putNumber("Shoot Speed U2", ShootSpeedUpper2);
        SmartDashboard.putNumber("Shoot Speed U3", ShootSpeedUpper3);
        SmartDashboard.putNumber("Shoot Speed U4", ShootSpeedUpper4);
        SmartDashboard.putNumber("Shoot Speed U5", ShootSpeedUpper5);


        SmartDashboard.putNumber("Shoot Speed L1", ShootSpeedLower1);
        SmartDashboard.putNumber("Shoot Speed L2", ShootSpeedLower2);
        SmartDashboard.putNumber("Shoot Speed L3", ShootSpeedLower3);
        SmartDashboard.putNumber("Shoot Speed L4", ShootSpeedLower4);
        SmartDashboard.putNumber("Shoot Speed L5", ShootSpeedLower5);

        SmartDashboard.putNumber("Shoot Rev Speed L", ShooterLowerRevSetSpeed);
        SmartDashboard.putNumber("Shoot Rev Speed U", ShooterUpperRevSetSpeed);

        SmartDashboard.putNumber("Top Conveyor kF", topConveyorKf);
        SmartDashboard.putNumber("Top Conveyor kP", topConveyorKp);
        SmartDashboard.putNumber("Top Conveyor ki", topConveyorKi);
        SmartDashboard.putNumber("Top Conveyor kizone", topConveyorKizone);

        SmartDashboard.putNumber("shooter kF", shooterKf);
        SmartDashboard.putNumber("shooter kP", shooterKp); 
        SmartDashboard.putNumber("shooter kI", shooterKi);
        SmartDashboard.putNumber("shooter kIzone", shooterKizone);

        SmartDashboard.putNumber("Shooter Top RPM",0);
        SmartDashboard.putNumber("Shooter Bot RPM", 0);
        SmartDashboard.putNumber("Top Conveyor RPM",0);    

        SmartDashboard.putNumber("Rev Shooter dt", 0.2);


        TopConveyor = new TalonFX(15);
        FrontIntakeBar = new TalonFX(10);
        RearIntakeBars = new TalonFX(11);
        ShooterTop = new TalonFX(12);
        ShooterBottom = new TalonFX(13);

        ShooterBottom.configFactoryDefault();
        ShooterTop.configFactoryDefault();
        RearIntakeBars.configFactoryDefault();
        FrontIntakeBar.configFactoryDefault();
        TopConveyor.configFactoryDefault();

        TopConveyor.config_kF(0,topConveyorKf );
        TopConveyor.config_kP(0,topConveyorKp );
        TopConveyor.config_kI(0,topConveyorKp );
        TopConveyor.config_IntegralZone(0,topConveyorKizone );

        ShooterBottom.config_kP(0,shooterKp);
        ShooterBottom.config_kF(0,shooterKf);
        ShooterBottom.config_kI(0,shooterKi);
        ShooterBottom.config_IntegralZone(0,shooterKizone);
        
        ShooterTop.config_kP(0,shooterKp);
        ShooterTop.config_kF(0,shooterKf);
        ShooterTop.config_kI(0,shooterKi);
        ShooterTop.config_IntegralZone(0,shooterKizone);

        ShooterTop.setNeutralMode(NeutralMode.Coast);
        ShooterBottom.setNeutralMode(NeutralMode.Coast);


        ShooterTop.setInverted(InvertType.InvertMotorOutput);
        FrontIntakeBar.setInverted(InvertType.InvertMotorOutput);

        ShooterTop.configVoltageCompSaturation(12.0);        
        ShooterBottom.configVoltageCompSaturation(12.0);
        


        irSensorL = new DigitalInput(1);        
    }



  public void reverseIntake() {
    TopConveyor.set(ControlMode.Velocity,-topConveyorIntakeSpeed);
//    FrontIntakeBar.set(ControlMode.PercentOutput, -cubeIntakePower);
//    RearIntakeBars.set(ControlMode.PercentOutput, -rearIntakePower);
    ShooterTop.set(ControlMode.Velocity, ShooterUpperRevSetSpeed);
    ShooterBottom.set(ControlMode.Velocity,ShooterLowerRevSetSpeed);
  }

  public void IntakeCube() {
      TopConveyor.set(ControlMode.Velocity,topConveyorIntakeSpeed);
      FrontIntakeBar.set(ControlMode.PercentOutput, cubeIntakePower);
      RearIntakeBars.set(ControlMode.PercentOutput, rearIntakePower);
  }

  public void Unstow(){
    FrontIntakeBar.set(ControlMode.PercentOutput, unstowRollerSpeed);
  }

  public void runFrontIntakeBar() {
    FrontIntakeBar.set(ControlMode.PercentOutput, frontIntakePower);
}

public void runFrontIntakeBar(double speed) {
  FrontIntakeBar.set(ControlMode.PercentOutput, speed);
}

  public void GrabCone() {
      FrontIntakeBar.set(ControlMode.PercentOutput, frontIntakePower);
      RearIntakeBars.set(ControlMode.PercentOutput, -rearIntakePower);
  }
  public void DropCone() {
      FrontIntakeBar.set(ControlMode.PercentOutput, -frontIntakePower);
      RearIntakeBars.set(ControlMode.PercentOutput, rearIntakePower);
  }

  public void injectCube(int speedLevel){
//    System.out.println("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
    double speed;
    if (speedLevel==1)speed=0.35;
    else speed=topConveyorInjectSpeed;
    TopConveyor.set(ControlMode.PercentOutput,speed);
  }

  public void stowRollers(){
    FrontIntakeBar.set(ControlMode.PercentOutput, stowRollerSpeed);

  }

  public double getFrontIntakeBarCurrent(){
    return FrontIntakeBar.getStatorCurrent();
  }

  public double getRearIntakeBarCurrent(){
    return RearIntakeBars.getStatorCurrent();
  }

  public boolean AtShootSpeed(){
    return (getBottomShooterRPM()>=0.9*ShooterLowerSetSpeed);
  }

  public void ShootCube(int level){
    if(level==1){
      ShooterUpperSetSpeed=ShootSpeedUpper1;
      ShooterLowerSetSpeed=ShootSpeedLower1;
      }
    else if(level==2){
      ShooterUpperSetSpeed=ShootSpeedUpper2;
      ShooterLowerSetSpeed=ShootSpeedLower2;
      }
    else if (level==3) {
//      System.out.println("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF");
      ShooterUpperSetSpeed=ShootSpeedUpper3;
      ShooterLowerSetSpeed=ShootSpeedLower3;
      }

    else if (level==4) {
//      System.out.println("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG");
        ShooterUpperSetSpeed=ShootSpeedUpper4;
        ShooterLowerSetSpeed=ShootSpeedLower4;
    }

    else {
      ShooterUpperSetSpeed=ShootSpeedUpper5;
      ShooterLowerSetSpeed=ShootSpeedLower5;
    }
      
    ShooterTop.set(ControlMode.Velocity, ShooterUpperSetSpeed);
    ShooterBottom.set(ControlMode.Velocity, ShooterLowerSetSpeed);
    SmartDashboard.putNumber("Shooter Top RPM", getTopShooterRPM());
    SmartDashboard.putNumber("Shooter Bot RPM", getBottomShooterRPM());
  }


  public void reverseShooter(){
    ShooterTop.set(ControlMode.Velocity, ShooterUpperRevSetSpeed);
    ShooterBottom.set(ControlMode.Velocity, ShooterLowerRevSetSpeed);
    TopConveyor.set(ControlMode.Velocity,topConveyorRevIntakeSpeed);
  }

  public double getBottomShooterRPM(){
    return  600*ShooterBottom.getSelectedSensorVelocity()/2048;
  }

  public double getTopShooterRPM(){
    return  600*ShooterTop.getSelectedSensorVelocity()/2048;
  }


  public double getTopConveyorSpeed(){
    return TopConveyor.getSelectedSensorVelocity();
  }

  public double getShooterCLE(){
    return ShooterBottom.getClosedLoopError();
  }

  public double getFrontRollerCurrent(){
    return FrontIntakeBar.getStatorCurrent();
  }



  public void StopMotors(){
    TopConveyor.set(ControlMode.PercentOutput, 0);
    ShooterBottom.set(ControlMode.PercentOutput, 0);
    ShooterTop.set(ControlMode.PercentOutput, 0);
    FrontIntakeBar.set(ControlMode.PercentOutput, 0);
    RearIntakeBars.set(ControlMode.PercentOutput, 0);
    }


    @Override
    public void periodic() {
      sensorValue=!irSensorL.get();
    }

  public boolean HasCube(){
    return sensorValue;
  }


  public void UpdateConstants() {
    topConveyorIntakeSpeed = SmartDashboard.getNumber("Top Conveyor Intake Speed",topConveyorIntakeSpeed);
    topConveyorInjectSpeed = SmartDashboard.getNumber("Top Conveyor Inject Speed",topConveyorInjectSpeed);
    topConveyorRevIntakeSpeed =  SmartDashboard.getNumber("Top Conveyor Intake Rev Speed",topConveyorRevIntakeSpeed);

    frontIntakePower = SmartDashboard.getNumber("Front Intake Power",frontIntakePower);
    rearIntakePower = SmartDashboard.getNumber("Rear Intake Power",rearIntakePower);
    stowRollerSpeed = SmartDashboard.getNumber("Stow Roller Speed",stowRollerSpeed);
    unstowRollerSpeed= SmartDashboard.getNumber("Unstow Roller Speed",unstowRollerSpeed);

    ShootSpeedLower1 = SmartDashboard.getNumber("Shoot Speed L1", ShootSpeedLower1);
    ShootSpeedLower2 = SmartDashboard.getNumber("Shoot Speed L2", ShootSpeedLower2);
    ShootSpeedLower3 = SmartDashboard.getNumber("Shoot Speed L3", ShootSpeedLower3);
    ShootSpeedLower4 = SmartDashboard.getNumber("Shoot Speed L5", ShootSpeedLower4);
    ShootSpeedLower5 = SmartDashboard.getNumber("Shoot Speed L5", ShootSpeedLower5);
    ShootSpeedUpper1 = SmartDashboard.getNumber("Shoot Speed U1", ShootSpeedUpper1);
    ShootSpeedUpper2 = SmartDashboard.getNumber("Shoot Speed U2", ShootSpeedUpper2);
    ShootSpeedUpper3 = SmartDashboard.getNumber("Shoot Speed U3", ShootSpeedUpper3);
    ShootSpeedUpper4 = SmartDashboard.getNumber("Shoot Speed U4", ShootSpeedUpper4);
    ShootSpeedUpper5 = SmartDashboard.getNumber("Shoot Speed U5", ShootSpeedUpper5);

    ShooterLowerRevSetSpeed =   SmartDashboard.getNumber("Shoot Rev Speed L", ShooterLowerRevSetSpeed);
    ShooterUpperRevSetSpeed =   SmartDashboard.getNumber("Shoot Rev Speed U", ShooterUpperRevSetSpeed);


    topConveyorKf = SmartDashboard.getNumber("Top Conveyor kF", topConveyorKf);
    topConveyorKp = SmartDashboard.getNumber("Top Conveyor kP", topConveyorKp);
    topConveyorKi = SmartDashboard.getNumber("Top Conveyor kI", topConveyorKi);
    topConveyorKizone = SmartDashboard.getNumber("Top Conveyor kIzone", topConveyorKizone);

    shooterKf = SmartDashboard.getNumber("shooter kF", shooterKf);
    shooterKp = SmartDashboard.getNumber("shooter kP", shooterKp);
    shooterKi = SmartDashboard.getNumber("shooter kI", shooterKi);
    shooterKizone = SmartDashboard.getNumber("shooter kIzone", shooterKizone);


    TopConveyor.config_kF(0, topConveyorKf);
    TopConveyor.config_kP(0,topConveyorKp);
    TopConveyor.config_kI(0, topConveyorKi);
    TopConveyor.config_IntegralZone(0,topConveyorKizone);

    ShooterBottom.config_kP(0,shooterKp);
    ShooterBottom.config_kF(0,shooterKf);
    ShooterBottom.config_kI(0,shooterKi);
    ShooterBottom.config_IntegralZone(0,shooterKizone);
    
    ShooterTop.config_kP(0,shooterKp);
    ShooterTop.config_kF(0,shooterKf);
    ShooterTop.config_kI(0,shooterKi);
    ShooterTop.config_IntegralZone(0,shooterKizone);
}

}
