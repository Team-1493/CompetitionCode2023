package frc.robot.subsystems;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sensors.Pigeon;

public class SwerveDrive extends SubsystemBase {

    public SwerveModule[] modules = new SwerveModule[4];
    // Robot Dimensions for MK4 Swerve
    public  double  maxVelocityFPS = 11.48;  //max speed in feet/sec
    public double maxVelocityMPS = 0.3048*maxVelocityFPS; // 3.5     


      
public static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  new Translation2d(0.2131, +0.2105), 
  new Translation2d(0.2131, -0.2105), 
  new Translation2d(-0.2131, +0.2105), 
  new Translation2d(-0.2131, -0.2105));

  public final Pigeon gyro = new Pigeon(20);
  public SwerveModuleState[] moduleStatesOptimized = new SwerveModuleState[4];
  public double heading=-gyro.getHeadingRadians();
  public SwerveModulePosition[] modulePos = new SwerveModulePosition[4];
  public SwerveDriveOdometry m_odometry ;
  public ProfiledPIDController rotatePID;
  private double kProtate=4.0,kDrotate=0.1;
  public double TrapMaxVel_rotate=20;
  public double TrapMaxAcc_rotate=30;
  private TrapezoidProfile.Constraints trapProf =
  new TrapezoidProfile.Constraints(TrapMaxVel_rotate,TrapMaxAcc_rotate);

    
  private double[] encPositionRad = new double[4];   // encoder position of swerve motors
  private String[] moduleNames={"FR","FL","BR","BL"};
  double headingSet=0;
  public boolean rotatePIDon=false;
  

 // Constrcutor 
  public SwerveDrive(){
    
    // Turn Module Offsets in degrees   FR-FL-BR-BL
    double[] turnMotorZeroPos={57.7,-85.4,117.2,-77.6};

    modules[0]=new SwerveModule("FR",2,1,11,
        turnMotorZeroPos[0]);
    modules[1]=new SwerveModule("FL",4,3,13,
        turnMotorZeroPos[1]);
    modules[2]=new SwerveModule("BR",6,5,15,
        turnMotorZeroPos[2]);
    modules[3]=new SwerveModule("BL",8,7,17,
        turnMotorZeroPos[3]);

    modulePos=getModulePositions();    
    m_odometry=new SwerveDriveOdometry(m_kinematics,new Rotation2d(heading),modulePos, 
        new Pose2d(0,0,new Rotation2d(0)));

        
    
    rotatePID=new ProfiledPIDController(kProtate, 0, kDrotate, trapProf);   // need to tune this
    rotatePID.setTolerance(.02);
    rotatePID.enableContinuousInput(-Math.PI, Math.PI);
    resetRotatePID(0);

    SmartDashboard.putNumber("Max Vel FPS",maxVelocityFPS);
    SmartDashboard.putNumber("Max Drive RPM",modules[0].MPStoRPM(maxVelocityMPS));
    SmartDashboard.putNumber("kProtate",kProtate);
    SmartDashboard.putNumber("kDrotate",kDrotate);

  }      


  // convert joystick magnitudes to velocity (mps) and rotational rate (rad/sec)
  // then call set mootors to those speeds      StickState Array:  vx,vy,omega 
  public void setMotorsFromStick(double[] stickState ) {
    double vx=stickState[0]*maxVelocityMPS;
    double vy=stickState[1]*maxVelocityMPS;
    double omega=2*stickState[2];

    if (Math.abs(omega)<0.001){
/* 
      if( !rotatePIDon) {
        headingSet=heading;
        rotatePIDon=true;
      }
      */
    if(rotatePIDon) setMotors(vx, vy);
    else setMotors(vx, vy,0);



    }
    else {
      rotatePIDon=false;
      setMotors(vx,vy,omega);
    }
    SmartDashboard.putBoolean("rotateMode", rotatePIDon);
    SmartDashboard.putNumber("heading set", headingSet);
}

//  set motors to provided vx,vy and an omega calculated from the rotation PID controller
// must have set the reset the controller and set its goal  
public void setMotors(double vx,double vy) {
  //  control will return immediately back to setMotorsFromStick. 
  // We don't want to change the heading setpoint. Robot will keep turning to setpoint
  // until rotate stick is pushed or this method is called again. 
  rotatePIDon=true;  
  double pidOutput= rotatePID.calculate(heading);
  SmartDashboard.putNumber("rotatePID calc",pidOutput);
  SmartDashboard.putNumber("heading error", rotatePID.getPositionError());
  setMotors(vx,vy,pidOutput);
}

// set motors using specified  vx,vy, omega in meters/sec and radians/sec
   public void setMotors(double vx,double vy, double omega ) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    vx, vy, omega,  new Rotation2d(heading));
// Convert to speeds module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
}





public void setModuleStates(SwerveModuleState[] moduleStates){
  // normalize wheel speed so no wheel exceeds max allowable velocity
  SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,maxVelocityMPS);
  int i = 0;
  while(i<4){
    // get the current turn encoder position in radians
    encPositionRad[i]=modules[i].getTurnPosition_Rad();
    // optimize module state to minimize the turn rotation needed
    moduleStatesOptimized[i]=optimize(moduleStates[i],encPositionRad[i]);
    // get the drive motor's setpoint in rpm 
    double speedSet=moduleStatesOptimized[i].speedMetersPerSecond;
    // get the turn motor's rotation setpoint radians
    double turnSet = moduleStatesOptimized[i].angle.getRadians();
    modules[i].setMotors(speedSet, turnSet);
    i++;  
  }
}

  public void allStop(){
    int i=0;
    while(i<4){
      modules[i].setMotorsAllStop();
      i++;
    }

  }


  public CommandBase stopRotateInPlace() {
    return this.runOnce(() -> rotatePIDon=false);
  }



// a bunch of getters - so that the everything except the SwerveModules class can be 
// independant of the type of motors being used
  

  public void resetEncoders() {
    int i = 0;
    while(i<4){
      modules[i].resetEncoders();
      i++;
    }
  }


  public void resetGyro() {
    gyro.resetAngle();
    resetOdometryToZero();
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

  public Command UpdateConstantsCommand() {
    // implicitly requires `this`
    return this.runOnce(() -> this.updateConstants());
  }

  public void updateConstants() {

    maxVelocityMPS = 0.3048*maxVelocityFPS; 
    SmartDashboard.putNumber("Max Drive RPM",modules[0].MPStoRPM(maxVelocityMPS));
    int i=0;
     while(i<4){
      modules[i].updateConstants();
      i++;
    } 
    kProtate=SmartDashboard.getNumber("kProtate", 0);
    kDrotate=SmartDashboard.getNumber("kDrotate", 0);
    rotatePID.setP(kProtate);
    rotatePID.setD(kDrotate);
  }

  public void setPIDSlot(int slot){
    int i=0;
    while(i<4){
      modules[i].setPIDslot(slot);
      i++;
    }
  }


  public double getDriveVelocityMagnitude() {
    return Math.abs(modules[0].getDriveVelocity());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

// set the gyro yaw to a new heading, angles in degrees
  public void setHeading(double newHeading){
    gyro.setAngle(newHeading);
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(new Rotation2d(heading),getModulePositions(),pose);
  }

  public void resetOdometryWithHeading(Pose2d pose,double newHeading) {
    m_odometry.resetPosition(new Rotation2d(newHeading),getModulePositions(),pose);
  }

public void resetOdometryToZero(){
  Pose2d zeroPose= new Pose2d(new Translation2d(0,0),new Rotation2d(0));
  m_odometry.resetPosition(new Rotation2d(0),getModulePositions(),zeroPose);
}


  @Override
  public void periodic() {
    heading=-gyro.getHeadingRadians();
    modulePos=getModulePositions();    
    try{
    m_odometry.update(
        new Rotation2d(heading),modulePos);
        SmartDashboard.putNumber("pose-rot", m_odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("pose-x", m_odometry.getPoseMeters().getX()*39.37);
        SmartDashboard.putNumber("pose-y", m_odometry.getPoseMeters().getY()*39.37);
        printModuleStates();
//        SmartDashboard.putNumber("module state 0 mps", modules[0].getState().speedMetersPerSecond);
      }
        catch(Exception e){
          
        }
  }


// print module values to smartdashboard
private void printModuleStates(){
int i=0;
while(i<4){
// add whatever values you want to see
  SmartDashboard.putNumber(moduleNames[i]+" Dpos",modules[i].getDrivePositionRotations());            
  SmartDashboard.putNumber(moduleNames[i]+" Dvel",modules[i].getDriveVelocity()); 
  SmartDashboard.putNumber(moduleNames[i]+" DCLE",modules[i].getDriveCLE()); 

  SmartDashboard.putNumber(moduleNames[i]+" TPos",modules[i].getTurnPosition_Deg());
  SmartDashboard.putNumber(moduleNames[i]+" TabsPos",modules[i].getTurnAbsPosition());

  i++;
}
SmartDashboard.putNumber("Heading",heading);
SmartDashboard.putNumber("PIDRotate Error",rotatePID.getPositionError());

}



//   ******  This is changed from last year !!!  Not certain this is correct !!!
public SwerveModulePosition[] getModulePositions(){
  SwerveModulePosition[] positions = new SwerveModulePosition[4];
  positions[0]=new SwerveModulePosition(
      modules[0].getDrivePositionMeters(),
      new Rotation2d(modules[0].getTurnPosition_Rad())) ;
  positions[1]=new SwerveModulePosition(
      modules[1].getDrivePositionMeters(),
      new Rotation2d(modules[1].getTurnPosition_Rad())) ;
  positions[2]=new SwerveModulePosition(
        modules[2].getDrivePositionMeters(),
        new Rotation2d(modules[2].getTurnPosition_Rad())) ;
  positions[3]=new SwerveModulePosition(
      modules[1].getDrivePositionMeters(),
      new Rotation2d(modules[3].getTurnPosition_Rad())) ;
  return positions;
}


static public  SwerveModuleState optimize(SwerveModuleState sms, double currentAngle){
    double twoPi = 2*Math.PI;
    double threePiover2 = 3*Math.PI/2;
    double Piover2=Math.PI/2;
    double angleTarget=sms.angle.getRadians();
    double anglediff=angleTarget-currentAngle%twoPi;
    double angleChange=0;
    double optimizedAngle;
    int rev=1;
    if (anglediff>threePiover2) anglediff=anglediff-twoPi;
    if (anglediff<-threePiover2) anglediff=anglediff+twoPi;

    if (anglediff >Piover2 && anglediff < threePiover2 ) {
        angleChange=anglediff-Math.PI;
        rev=-1;}
    else if (anglediff <-Piover2 && anglediff > -threePiover2 ) {
        angleChange=anglediff+Math.PI;
        rev=-1;}    
    else angleChange=anglediff;    
    optimizedAngle=angleChange+currentAngle;
    double speed = sms.speedMetersPerSecond*rev;    
    return new SwerveModuleState(
        speed, new Rotation2d(optimizedAngle));
}

public void resetRotatePID(double goal){
  rotatePID.reset(heading);
  rotatePID.setGoal(goal);
}

}