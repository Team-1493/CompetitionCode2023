package frc.robot.subsystems;
import java.time.Instant;
// comment added from computer 12 for github test
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CubeIntakeAuto;
import frc.robot.commands.ResetOdometryAuto;
import frc.robot.commands.ShootCubeAuto;
import frc.robot.commands.Stow;
import frc.robot.subsystems.CustomPPS;
import frc.robot.commands.autobalancer2;


public class AutoGenerator extends SubsystemBase{
    //Defining the SwerveDrive subsystem used by the AutoGenerator in a placeholder variable
    private SwerveDrive sds;   
    private IntakeSystem intake;
    
    private ResetOdometryAuto resetRedRightBal1;

    //Defining the CubeIntake and Stow commands used by the AutoGenerator in placeholder variables
    private CubeIntakeAuto intakeCube;
    private Stow stowArm;

    private int closeHighSpeed = 3; //placeholder
    private int closeMidSpeed = 2; //placeholder shoot to mid close
    private int farHighSpeed = 5; //placeholder
    private int farLowSpeed = 7; //changed from 4 to 7


    private InstantCommand injectCube;
    private InstantCommand endShoot;
    private InstantCommand cancelIntakeCube;
    private ShootCubeAuto shootCloseHigh;
    private ShootCubeAuto shootCloseMid;
    private ShootCubeAuto shootFarHigh;
    private ShootCubeAuto shootFarLow;
    private ShootCubeAuto shootFarLowEnd;
    private ShootCubeAuto shootFarMid;
    private ShootCubeAuto shootFarMid2,shootFarMid3,shootFarMid4,shootFarMid5;

    private ShootCubeAuto shootFarHighInitial1, shootFarHighInitial2, shootFarHighInitial3;
    private ShootCubeAuto shootFarHighInitial4, shootFarHighInitial5, shootFarHighInitial6;
    private ShootCubeAuto shootFarHighInitial7;
    private ShootCubeAuto shootCloseHighInitial1,shootCloseHighInitial2,shootCloseHighInitial3;
    private ShootCubeAuto shootCloseHighInitial4,shootCloseHighInitial5,shootCloseHighInitial6;
    private ShootCubeAuto shootCloseHighInitial7,shootCloseHighInitial8;
    private ShootCubeAuto shootCloseHighInitial9,shootCloseHighInitial10;



    private ShootCubeAuto shootCloseMidEnd1,shootCloseMidEnd2;
    private ShootCubeAuto shootCloseMidEnd3,shootCloseMidEnd4;
 
     private autobalancer2 bal1;
     private autobalancer2 balShooter1,balShooter2,balShooter3,
            balShooter4,balShooter5,balShooter6;
    private autobalancer2 balShooter7,balShooter8,balShooter9;
        


     


    //Defining a HashMap called eventMap, which will store all events that can run during auto
    private HashMap<String, Command> eventMap = new HashMap<>();
    
    
    //Loading all autonomous paths and storing them in variables
//    public PathPlannerTrajectory testPath1 = PathPlanner.loadPath("testPath1", new PathConstraints(4, 3));
    public PathPlannerTrajectory trajTestPath1 = PathPlanner.loadPath(
        "testPath1", new PathConstraints(.75, .75));

    public PathPlannerTrajectory trajBalanceFromMiddle = PathPlanner.loadPath(
        "BalanceFromMiddle", new PathConstraints(1, 1));
       

// ************
    public PathPlannerTrajectory trajRedLeft2 = PathPlanner.loadPath(
        "pathRedLeft2", new PathConstraints(1.5, 1.5));
   
    public PathPlannerTrajectory trajBlueLeft2 = PathPlanner.loadPath(
        "pathBlueLeft2", new PathConstraints(1.5, 1.5));
    

// *************
    public PathPlannerTrajectory trajRedLeft2Bal = PathPlanner.loadPath(
        "pathRedLeft2Bal", new PathConstraints(1.75, 1.75));
    
    public PathPlannerTrajectory trajBlueLeft2Bal = PathPlanner.loadPath(
        "pathBlueLeft2Bal", new PathConstraints(1.75, 1.75));
    
        
// *************
    public PathPlannerTrajectory trajRedLeftBal1 = PathPlanner.loadPath(
        "pathRedLeftBal1", new PathConstraints(1.75, 1.25));
    
    public PathPlannerTrajectory trajBlueLeftBal1 = PathPlanner.loadPath(
            "pathBlueLeftBal1", new PathConstraints(1.75, 1.25));
    

// *************
    public PathPlannerTrajectory trajRedLeftReturn2 = PathPlanner.loadPath(
        "pathRedLeftReturn2", new PathConstraints(1.5, 1.5));
    
    public PathPlannerTrajectory trajBlueLeftReturn2 = PathPlanner.loadPath(
        "pathBlueLeftReturn2", new PathConstraints(1.5, 1.5));

        
   // *************     
   public PathPlannerTrajectory trajRedRightBal3 = PathPlanner.loadPath(
    "pathRedRightBal3", new PathConstraints(2.0, 1.5));           
    public PathPlannerTrajectory trajBlueRightBal3 = PathPlanner.loadPath(
        "pathBlueRightBal3", new PathConstraints(2.0, 1.5));           
   
   
// *************     
   public PathPlannerTrajectory trajRedRight3 = PathPlanner.loadPath(
    "pathRedRight3", new PathConstraints(1.75, 1.75)); 
    
    public PathPlannerTrajectory trajBlueRight3 = PathPlanner.loadPath(
        "pathBlueRight3", new PathConstraints(1.75, 1.75)); 


// ******************************
   public PathPlannerTrajectory trajRedRightBal2 = PathPlanner.loadPath(
        "pathRedRightBal2", new PathConstraints(1.5, 1.5));   
    public PathPlannerTrajectory trajBlueRightBal2 = PathPlanner.loadPath(
            "pathBlueRightBal2", new PathConstraints(1.5, 1.5));   
   
        



    //Creates a path using the robot's initial position (from sds) and the desired position (given by vision)
    public PathPlannerTrajectory getPathUsingVision(Translation2d end_pose, Double end_heading, Double end_rotation){
        return PathPlanner.generatePath(
            new PathConstraints(4,3),
            new PathPoint(new Translation2d(sds.getPose().getX(), sds.getPose().getY()), //Creates a point (starting_position(x,y) (from sds), starting_direction(assumed 0), starting_rotation (from sds))
                Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(sds.getPose().getRotation().getDegrees())),
            new PathPoint(end_pose, Rotation2d.fromDegrees(end_heading), Rotation2d.fromDegrees(end_rotation)) //Creates a point using the given values. These values will be determined by vision
            );
    }
    
 

    //This method will be called once during the beginning of autonomous
    public AutoGenerator(SwerveDrive m_sds, ArmSubsystem arm, IntakeSystem m_intake) {
        //defining the SwerveDrive used by this class as the given SwerveDrive instance
        sds = m_sds;
        intake=m_intake;

     
        shootCloseHigh =new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseMid = new ShootCubeAuto(intake, closeMidSpeed);
        shootFarHigh= new ShootCubeAuto(intake, farHighSpeed);
        shootFarLow= new ShootCubeAuto(intake, farLowSpeed);

        shootFarMid2 = new ShootCubeAuto(intake, 6);
        shootFarMid3 = new ShootCubeAuto(intake, 6);
        shootFarMid4 = new ShootCubeAuto(intake, 6);
        shootFarMid5 = new ShootCubeAuto(intake, 6);

        
        shootFarLowEnd= new ShootCubeAuto(intake, farLowSpeed);
    
        shootFarHighInitial1=new ShootCubeAuto(intake, farLowSpeed);
        shootFarHighInitial2=new ShootCubeAuto(intake, farLowSpeed);
        shootFarHighInitial3=new ShootCubeAuto(intake, farLowSpeed);
        shootFarHighInitial4=new ShootCubeAuto(intake, farLowSpeed);
        shootFarHighInitial5=new ShootCubeAuto(intake, farLowSpeed);
        shootFarHighInitial6=new ShootCubeAuto(intake, farLowSpeed);
        shootFarHighInitial7=new ShootCubeAuto(intake, farLowSpeed);
        
        shootCloseHighInitial1=new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseHighInitial2=new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseHighInitial3=new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseHighInitial4=new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseHighInitial5=new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseHighInitial6=new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseHighInitial7=new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseHighInitial8=new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseHighInitial9=new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseHighInitial10=new ShootCubeAuto(intake, closeHighSpeed);





        shootCloseMidEnd1=new ShootCubeAuto(intake, 6);
        shootCloseMidEnd2=new ShootCubeAuto(intake, 6);
        shootCloseMidEnd3=new ShootCubeAuto(intake, 6);
        shootCloseMidEnd4=new ShootCubeAuto(intake, 6);
        shootFarMid=new ShootCubeAuto(intake, 6);
     
         bal1 = new autobalancer2(sds,1);

         balShooter1 = new autobalancer2(sds,1);
         balShooter2 = new autobalancer2(sds,1);
         balShooter3 = new autobalancer2(sds,1);
         balShooter4 = new autobalancer2(sds,1);
         balShooter5 = new autobalancer2(sds,1);
         balShooter6 = new autobalancer2(sds,1);
         balShooter7 = new autobalancer2(sds,1);
         balShooter8 = new autobalancer2(sds,1);
         balShooter9 = new autobalancer2(sds,1);
    
   

        //defining the CubeIntake and Stow commands used by this class by using the given ArmSubsystem and IntakeSystem
        intakeCube = new CubeIntakeAuto(arm, intake);
        cancelIntakeCube=new InstantCommand(() -> intakeCube.cancel());
        stowArm = new Stow(arm, intake);
        injectCube = new InstantCommand(() -> intake.injectCube(0) );
        endShoot = new InstantCommand(() -> intake.StopMotors());


        //When 360 degrees is exceeded, the rotation will loop back to 1 (no going over 360 degrees or 2*PI radians)
       



        //Putting default values into the Smartdashboard for everything relating to auto paths
        // SmartDashboard.putBoolean("Intake_Is_On", false);
        // SmartDashboard.putNumber("Path_position", 0.00);
        // SmartDashboard.putString("Path3_position", "none");
        
        //These events are used in all or multiple autonomous paths
        eventMap.put("shoot_far_mid", shootFarMid);
        eventMap.put("intake_cube", intakeCube);
        eventMap.put("cancel_intake_cube", cancelIntakeCube);
        eventMap.put("stow", stowArm);
        eventMap.put("shoot_close_high", shootCloseHigh);
        eventMap.put("shoot_close_mid", shootCloseMid);
        eventMap.put("shoot_far_high", shootFarHigh);
        eventMap.put("shoot_far_low", shootFarLow);
        eventMap.put("inject_cube", injectCube);
        eventMap.put("endShoot", endShoot);

    
        
        //The multi-line comment below is for testing if needed
/*       
        double timeEnd = trajTestPath1.getEndState().timeSeconds;
        double t = 0;
        while  (t<timeEnd){
            PathPlannerState state = (PathPlannerState) trajTestPath1.sample(t);
            System.out.println(state.poseMeters.getX()+",  "+state.poseMeters.getY()+", "+
            state.holonomicRotation.getDegrees()+
            ","+state.poseMeters.getRotation().getDegrees()+", "+state.velocityMetersPerSecond);
            t=t+0.1;

        };

        System.out.println(trajTestPath1.getInitialHolonomicPose().getX()+"   "+
        trajRedLeftBal.getInitialHolonomicPose().getY()+"   "+
        trajRedLeftBal.getInitialHolonomicPose().getRotation().getDegrees() );
   */     
    }
    


    //Builds and returns a PPSwerveControllerCommand for the given path
    public CustomPPS buildSwerveControlCommand(PathPlannerTrajectory retrievedPath) {
        PIDController thetaController = new PIDController(2, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return new CustomPPS(
            retrievedPath, //the given path, which will be run
            sds::getPose,
            SwerveDrive.m_kinematics, 
            new PIDController(.75, 0,0 ), //x PID controller
            new PIDController(.75, 0,0), //y PID controller
            thetaController, //rotation PID controller
            sds::setModuleStates, 
            false, //if the robot is on the red alliance, the path will be reflected
            sds //the AutoGenerator's instance of the SwerveDrive
        );
    }

    //Builds a FollowPathWithEvents using a given PathPlannerTrajectory
    public FollowPathWithEvents followEventBuilder(PathPlannerTrajectory retrievedPath) {
        return new FollowPathWithEvents(
            buildSwerveControlCommand(retrievedPath),
            retrievedPath.getMarkers(),
            eventMap
        );
    } 

    //This method will set all PID values (kP, kI, kD) to the values in the SmartDashboard


    //This is a list commands to run during autonomous if testPath1 is being run
    public SequentialCommandGroup testAutoCommand1() {
        return new SequentialCommandGroup(
//            shootCloseHighInitial1.asProxy(),
            new InstantCommand( () -> sds.resetOdometry(trajTestPath1.getInitialHolonomicPose())),
            followEventBuilder(trajTestPath1),
            //bal1.asProxy(),
            new InstantCommand( () -> sds.allStop())
//            shootCloseMidEndTest.asProxy()
        );
    }


// *************
    public SequentialCommandGroup autoBalanceFromMiddle(){
        return new SequentialCommandGroup(
            shootCloseHighInitial1,
            new InstantCommand( () -> sds.resetOdometry(trajBalanceFromMiddle.getInitialHolonomicPose())),            
            followEventBuilder(trajBalanceFromMiddle),
            bal1,
            new InstantCommand( () -> sds.allStop())                
        );
}


// *************
    public SequentialCommandGroup autoShootHigh(){
        return new SequentialCommandGroup(
            shootCloseHighInitial8,
            new InstantCommand( () -> sds.allStop())
        );
    }

// *******************
    public SequentialCommandGroup autoRedLeft2(){
        return new SequentialCommandGroup(
            shootFarHighInitial2,
            new InstantCommand( () -> sds.resetOdometry(trajRedLeft2.getInitialHolonomicPose())),            
            followEventBuilder(trajRedLeft2),
            new InstantCommand( () -> sds.allStop()),
            shootFarMid2
        );
    }
    public SequentialCommandGroup autoBlueLeft2(){
        return new SequentialCommandGroup(
            shootFarHighInitial4,
            new InstantCommand( () -> sds.resetOdometry(trajBlueLeft2.getInitialHolonomicPose())),            
            followEventBuilder(trajBlueLeft2),
            new InstantCommand( () -> sds.allStop()),
            shootFarMid4
        );
    }


// *******************
    public SequentialCommandGroup autoRedLeft2Bal(){
        return new SequentialCommandGroup(
            shootFarHighInitial3,
            new InstantCommand( () -> sds.resetOdometry(trajRedLeft2Bal.getInitialHolonomicPose())),            
            followEventBuilder(trajRedLeft2Bal),
            balShooter5,
            new InstantCommand( () -> sds.allStop())
        );
    }

    public SequentialCommandGroup autoBlueLeft2Bal(){
        return new SequentialCommandGroup(
            shootFarHighInitial6,
            new InstantCommand( () -> sds.resetOdometry(trajRedLeft2Bal.getInitialHolonomicPose())),            
            followEventBuilder(trajBlueLeft2Bal),
            balShooter4,
            new InstantCommand( () -> sds.allStop())
        );
    }    



// *******************    
    public SequentialCommandGroup autoRedLeftBal1(){
        return new SequentialCommandGroup(
            shootCloseHighInitial2,
            new InstantCommand( () -> sds.resetOdometry(trajRedLeftBal1.getInitialHolonomicPose())),            
            followEventBuilder(trajRedLeftBal1),
            new InstantCommand( () -> sds.allStop()),
            balShooter9,
            shootCloseMidEnd2
            );
    }

    public SequentialCommandGroup autoBlueLeftBal1(){
        return new SequentialCommandGroup(
            shootCloseHighInitial9,
            new InstantCommand( () -> sds.resetOdometry(trajBlueLeftBal1.getInitialHolonomicPose())),            
            followEventBuilder(trajBlueLeftBal1),
            new InstantCommand( () -> sds.allStop()),
            balShooter6,
            shootCloseMidEnd3
            );
    }   


// *******************
public SequentialCommandGroup autoRedLeftReturn2(){
        return new SequentialCommandGroup(
            shootFarHighInitial7,
            new InstantCommand( () -> sds.resetOdometry(trajRedLeftReturn2.getInitialHolonomicPose())),            
            followEventBuilder(trajRedLeftReturn2),
            new InstantCommand( () -> sds.allStop()),
            shootCloseMidEnd1
            );
    }

    public SequentialCommandGroup autoBlueLeftReturn2(){
        return new SequentialCommandGroup(
            shootFarHighInitial5,
            new InstantCommand( () -> sds. resetOdometry(trajBlueLeftReturn2.getInitialHolonomicPose())),            
            followEventBuilder(trajBlueLeftReturn2),
            new InstantCommand( () -> sds.allStop()),
            shootCloseMidEnd4
            );
    }    



// *************
public SequentialCommandGroup autoRedRightBal3(){
    return new SequentialCommandGroup(
        shootCloseHighInitial5,
        new InstantCommand( () -> sds.resetOdometry(trajRedRightBal3.getInitialHolonomicPose())),            
        followEventBuilder(trajRedRightBal3),
        new InstantCommand( () -> sds.allStop()),
        balShooter3
        );
}

public SequentialCommandGroup autoBlueRightBal3(){
    return new SequentialCommandGroup(
        shootCloseHighInitial3,
        new InstantCommand( () -> sds.resetOdometry(trajBlueRightBal3.getInitialHolonomicPose())),            
        followEventBuilder(trajBlueRightBal3),
        new InstantCommand( () -> sds.allStop()),
        balShooter7
        );
}


//************************ 
public SequentialCommandGroup autoRedRight3(){
    return new SequentialCommandGroup(
        shootCloseHighInitial7,
        new InstantCommand( () -> sds.resetOdometry(trajRedRight3.getInitialHolonomicPose())),            
        followEventBuilder(trajRedRight3),
        new InstantCommand( () -> sds.allStop()),
        shootFarMid3
        );
}

public SequentialCommandGroup autoBlueRight3(){
    return new SequentialCommandGroup(
        shootCloseHighInitial6,
        new InstantCommand( () -> sds.resetOdometry(trajBlueRight3.getInitialHolonomicPose())),            
        followEventBuilder(trajBlueRight3),
        new InstantCommand( () -> sds.allStop()),
        shootFarMid5
        );
}


//  **********************************
    public SequentialCommandGroup autoRedRightBal2(){
        return new SequentialCommandGroup(
            shootCloseHighInitial4,
            new InstantCommand( () -> sds.resetOdometry(trajRedRightBal2.getInitialHolonomicPose())),            
            followEventBuilder(trajRedRightBal2),
            new InstantCommand( () -> sds.allStop()),
            balShooter2
            );
    }
    public SequentialCommandGroup autoBlueRightBal2(){
        return new SequentialCommandGroup(
            shootCloseHighInitial10,
            new InstantCommand( () -> sds.resetOdometry(trajBlueRightBal2.getInitialHolonomicPose())),            
            followEventBuilder(trajBlueRightBal2),
            new InstantCommand( () -> sds.allStop()),
            balShooter8
            );
    }
    





    //Runs a path from the robot's current position to a new position (given by vision)
    //end_pose = x and y,   end_heading = angle of movement in degrees (look at pathplanner),   end_rotation = robot's rotation in degrees 
    public SequentialCommandGroup autoVisionCommand(Translation2d end_pose, Double end_heading, Double end_rotation){
        PathPlannerTrajectory autoVisionPath = getPathUsingVision(end_pose, end_heading, end_rotation);
        return new SequentialCommandGroup(
            new InstantCommand(()-> sds.resetOdometry(autoVisionPath.getInitialHolonomicPose())),
            followEventBuilder(autoVisionPath),
            new InstantCommand(() -> sds.allStop())
        );
    }

}

