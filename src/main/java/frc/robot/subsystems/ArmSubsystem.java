// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CubeIntake;

public class ArmSubsystem extends SubsystemBase {
    TalonFX armMotor = new TalonFX(9);
    CANCoder enc = new CANCoder(19);
    DigitalInput limitUpper = new DigitalInput(0);
    DigitalInput limitLower = new DigitalInput(4);
    // Gains for position control
    double armkP = 0.3, armkD = 100, armkI = 0.00075, armkIZone=100, armkF = 5;
    double armkP2 =0.75, armkD2 = 75, armkI2 = 0.0, armkIZone2=100, armkF2 =3;
    double magicVel=75,magicAcc=75;

    // feedforward accounts for gravity, friction and s g
    // 3 sets of gains, depending on where arm is relative to neutral position -69 deg
    // region 1, -69 to 0 deg   
    // region 2, -69 to -90 deg
    // region 3, >0 
    double armkG1 = 0.01, armkS1 = 0.0;
    
    double armForwardSensorLim = 2600, armReverseSensorLim = 1140;
    double armMaxOutput = .15;
    public double posStow=1130,posStowFinish=1120;
    public double posCubeIntake=1330,posConeGrab=2000,posConePlace=2300;
    public double posOverCone=2000;
    public double setPoint=posStow;
    public double angleCounts;
    double angleOffset=-96;
    double angle;
    double time = 0;
    double arbff;
    int button = 0,slot=0;

    boolean ls_upper = true, ls_lower = false;
    boolean lsState=false;
    public boolean motorActive=false;
    

    public ArmSubsystem() {

        SmartDashboard.putNumber("arbff cubein",-.01);
        enc.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        enc.setPositionToAbsolute(20);
        enc.configSensorDirection(true);
        enc.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 5, 20);

        armMotor.configFactoryDefault();
        armMotor.configNeutralDeadband(0.001);

        // set voltage compensation
        armMotor.configVoltageCompSaturation(11.5);
        armMotor.enableVoltageCompensation(true);

        // Set encoder values as limit switch
        armMotor.configForwardSoftLimitThreshold(armForwardSensorLim);
        armMotor.configReverseSoftLimitThreshold(armReverseSensorLim);
        armMotor.configForwardSoftLimitEnable(true);
        armMotor.configReverseSoftLimitEnable(true);

        // set to brake mode
        armMotor.setNeutralMode(NeutralMode.Brake);

        // set feedbak sensor to cancoder
        armMotor.configRemoteFeedbackFilter(enc, 0);
        armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 20);

        // set max closed loop output
        armMotor.configClosedLoopPeakOutput(0, armMaxOutput);
        armMotor.configPeakOutputForward(armMaxOutput);
        armMotor.configPeakOutputReverse(-armMaxOutput);

        armMotor.configMotionAcceleration(magicAcc);
        armMotor.configMotionCruiseVelocity(magicVel);

    // set current limit
        StatorCurrentLimitConfiguration currentConfig = 
        new StatorCurrentLimitConfiguration(true, 30, 
        32, .1);
        armMotor.configStatorCurrentLimit(currentConfig);

        armMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 5);
        armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5);

        SmartDashboard.putNumber("arm kP", armkP);
        SmartDashboard.putNumber("arm kI", armkI);
        SmartDashboard.putNumber("arm kD", armkD);
        SmartDashboard.putNumber("arm kF", armkF);
        SmartDashboard.putNumber("arm kIzone", armkIZone);

        SmartDashboard.putNumber("arm kP2", armkP2);
        SmartDashboard.putNumber("arm kI2", armkI2);
        SmartDashboard.putNumber("arm kD2", armkD2);
        SmartDashboard.putNumber("arm kF2", armkF2);
        SmartDashboard.putNumber("arm kIzone2", armkIZone2);

        SmartDashboard.putNumber("arm Pos Stow", posStow);
        SmartDashboard.putNumber("arm Pos StowFinish", posStowFinish);
        SmartDashboard.putNumber("arm Pos Cube", posCubeIntake);
        SmartDashboard.putNumber("arm Pos Cone Grab", posConeGrab);
        SmartDashboard.putNumber("arm Pos Over Cone", posOverCone);
        SmartDashboard.putNumber("arm Pos Cone Place", posConePlace);

        SmartDashboard.putNumber("arm kG1", armkG1);
        SmartDashboard.putNumber("arm kS1", armkS1);


        SmartDashboard.putNumber("arm MagicAcc", magicAcc);
        SmartDashboard.putNumber("arm MagicVel", magicVel);


        SmartDashboard.putNumber("arm ForSensorLim", armForwardSensorLim);
        SmartDashboard.putNumber("arm RevSensorLim", armReverseSensorLim);
        SmartDashboard.putNumber("arm MaxOutput", armMaxOutput);
        updateConstants();

    }

    @Override
    public void periodic() {
        armMotor.getSelectedSensorPosition();
        angleCounts = armMotor.getSelectedSensorPosition(); 
        angle=angleCounts*360/4096+angleOffset;
        ls_upper=limitUpper.get();
        ls_lower=limitLower.get();
        lsState=!ls_upper;

        SmartDashboard.putNumber("arm voltage", armMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("arm current", armMotor.getStatorCurrent());
        SmartDashboard.putNumber("arm Pos Counts", angleCounts);
        SmartDashboard.putNumber("arm Angle", angle);
        SmartDashboard.putNumber("arm CLE", armMotor.getClosedLoopError());
        SmartDashboard.putBoolean("arm Upper LS", ls_upper);
        SmartDashboard.putBoolean("arm Lower LS", ls_lower);
    }


  

    public void resetIntegralAccumulator(){
        armMotor.setIntegralAccumulator(0, 0, 20);
    }

    public void setPositionInCounts(double counts) {    
        setPoint=counts;
        
        // slot 0 for high position gains, slot 2 for low position gains
        if(setPoint==posCubeIntake && slot!=0) {
            armMotor.selectProfileSlot(2, 0);
            slot=0;
        }

        else if (setPoint==posStow && slot!=1){
            armMotor.selectProfileSlot(0, 0);
            slot=1;
        }


        else if (setPoint==posOverCone && slot!=2){
            armMotor.selectProfileSlot(0, 0);
            slot=0;
        }
        calculateFeedfwd();
        if(!lsState && motorActive)
        armMotor.set(ControlMode.MotionMagic, counts, 
            DemandType.ArbitraryFeedForward, arbff);       
//      armMotor.set(ControlMode.Position, counts, DemandType.ArbitraryFeedForward, arbff);
        else StopMotors();
    }

    public void setArmWithStick(double value) {
        if (!ls_upper && value>0) {
            armMotor.set(ControlMode.PercentOutput, 0);
            motorActive=false;
        }
        else {
            armMotor.set(ControlMode.PercentOutput, value);
            motorActive=true;
        }
    }

    public void StopMotors() {
        motorActive=false;
        armMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean getLowerLimitSwitch(){
        return ls_lower;
    }

    public boolean getIsMotorActive(){
        return motorActive;
    }


// region 1: feedforward is positive   counts>=1377
// region 2: feedforward is negative   counts<=1309
// region 3:  mushy zone, interpolate  1309 < counts < 1377
    public void calculateFeedfwd(){
        double cosAngle=Math.cos(Math.PI*angle/180);
        arbff=armkG1*cosAngle+armkS1;
//        if (setPoint==posCubeIntake) arbff= SmartDashboard.getNumber("arbff cubein",0);
    }


    public double getCounts() {
        return angleCounts;
    }


    public void updateConstants() {
        armkG1 = SmartDashboard.getNumber("arm kG1", 0);
        armkS1 = SmartDashboard.getNumber("arm kS1", 0);

        armkP = SmartDashboard.getNumber("arm kP", 0);
        armkI = SmartDashboard.getNumber("arm kI", 0);
        armkD = SmartDashboard.getNumber("arm kD", 0);
        armkF = SmartDashboard.getNumber("arm kF", 0);
        armkIZone = SmartDashboard.getNumber("arm kIzone", 0);

        armkP2 = SmartDashboard.getNumber("arm kP2", 0);
        armkI2 = SmartDashboard.getNumber("arm kI2", 0);
        armkD2 = SmartDashboard.getNumber("arm kD2", 0);
        armkF2 = SmartDashboard.getNumber("arm kF2", 0);
        armkIZone2 = SmartDashboard.getNumber("arm kIzone2", 0);

        magicAcc=SmartDashboard.getNumber("arm MagicAcc", magicAcc);
        magicVel=SmartDashboard.getNumber("arm MagicVel", magicVel);

        SmartDashboard.putNumber("arm MagicVel", magicVel);
        SmartDashboard.putNumber("arm MagicAcc", magicAcc);


        armMotor.config_kP(0, armkP);
        armMotor.config_kI(0, armkI);
        armMotor.config_kD(0, armkD);
        armMotor.config_kF(0, armkF);
        armMotor.config_IntegralZone(0, armkIZone);

        armMotor.config_kP(2, armkP2);
        armMotor.config_kI(2, armkI2);
        armMotor.config_kD(2, armkD2);
        armMotor.config_kF(2, armkF2);
        armMotor.config_IntegralZone(0, armkIZone2);

        armMotor.config_kP(1, 1);
        armMotor.config_kI(1, 0);
        armMotor.config_kD(1, 75);
        armMotor.config_kF(1, 2);
        armMotor.config_IntegralZone(0, 0);


        armMotor.configMotionAcceleration(magicAcc);
        armMotor.configMotionCruiseVelocity(magicVel);


        armForwardSensorLim = SmartDashboard.getNumber("arm ForSensorLim", 0);
        armReverseSensorLim = SmartDashboard.getNumber("arm RevSensorLim", 0);
        armMotor.configForwardSoftLimitThreshold(armForwardSensorLim);
        armMotor.configReverseSoftLimitThreshold(armReverseSensorLim);

        armMaxOutput = SmartDashboard.getNumber("arm MaxOutput", 0);
        armMotor.configClosedLoopPeakOutput(0, armMaxOutput);
        armMotor.configPeakOutputForward(armMaxOutput);
        armMotor.configPeakOutputReverse(-armMaxOutput);

        posStow=SmartDashboard.getNumber("arm Pos Stow", posStow);
        posStowFinish=SmartDashboard.getNumber("arm Pos StowFinish", posStowFinish);
        posCubeIntake=SmartDashboard.getNumber("arm Pos Cube", posCubeIntake);
        posConeGrab=SmartDashboard.getNumber("arm Pos Cone Grab", posConeGrab);
        posOverCone=SmartDashboard.getNumber("arm Pos Over Cone", posOverCone);
        posConePlace=SmartDashboard.getNumber("arm Pos Cone Place", posConePlace);
    }

}
