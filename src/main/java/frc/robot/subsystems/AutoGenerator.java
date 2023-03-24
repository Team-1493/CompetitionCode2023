package frc.robot.subsystems;
import java.time.Instant;
// comment added from computer 12 for github test
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CubeIntakeAuto;
import frc.robot.commands.ShootCubeAuto;
import frc.robot.commands.Stow;
import frc.robot.commands.autobalancer2;


public class AutoGenerator extends SubsystemBase{
    //Defining the SwerveDrive subsystem used by the AutoGenerator in a placeholder variable
    private SwerveDrive sds;   
    private IntakeSystem intake;
    
    //Defining the CubeIntake and Stow commands used by the AutoGenerator in placeholder variables
    private CubeIntakeAuto intakeCube;
    private Stow stowArm;

    private int closeHighSpeed = 3; //placeholder
    private int closeLowSpeed = 2; //placeholder shoot to mid close
    private int farHighSpeed = 5; //placeholder
    private int farLowSpeed = 4; //placeholder


    private ShootCubeAuto shootCloseHigh;
    private ShootCubeAuto shootCloseLow;
    private ShootCubeAuto shootFarHigh;
    private ShootCubeAuto shootFarLow;
    private ShootCubeAuto shootFarLowEnd;

    private InstantCommand prepareCloseHigh;
    private InstantCommand prepareCloseLow;
    private InstantCommand prepareFarHigh;
    private InstantCommand prepareFarLow;
    private InstantCommand injectCube;
    private InstantCommand endShoot;
    private InstantCommand cancelIntakeCube;
    private ShootCubeAuto shootCloseHighInitial;
    private ShootCubeAuto shootCloseMidEnd;
    private autobalancer2 balance;

    //Defining a HashMap called eventMap, which will store all events that can run during auto
    private HashMap<String, Command> eventMap = new HashMap<>();
    
    
    //Loading all autonomous paths and storing them in variables
//    public PathPlannerTrajectory testPath1 = PathPlanner.loadPath("testPath1", new PathConstraints(4, 3));
    public PathPlannerTrajectory trajTestPath1 = PathPlanner.loadPath(
        "testPath1", new PathConstraints(2.2, 2.2));
    

    public PathPlannerTrajectory trajRedRightBal = PathPlanner.loadPath(
        "pathRedRightBal", new PathConstraints(2.5, 2.5));

    public PathPlannerTrajectory trajRedLeftBal = PathPlanner.loadPath(
        "pathRedLeftBal", new PathConstraints(2.2, 2.2));
    
    public PathPlannerTrajectory trajBlueRightBal = PathPlanner.loadPath(
         "pathBlueRightBal", new PathConstraints(2.2, 2.2));
        
    public PathPlannerTrajectory trajBlueLeftBal = PathPlanner.loadPath(
        "pathBlueLeftBal", new PathConstraints(2.5, 2.5));
    
    public PathPlannerTrajectory trajRedRight = PathPlanner.loadPath(
        "pathRedRight", new PathConstraints(2.5, 2.5));

    public PathPlannerTrajectory trajRedLeft = PathPlanner.loadPath(
        "pathRedLeft", new PathConstraints(2.5, 2.5));
    
     public PathPlannerTrajectory trajBlueRight = PathPlanner.loadPath(
        "pathBlueRight", new PathConstraints(2.2, 2.2));
            
    public PathPlannerTrajectory trajBlueLeft = PathPlanner.loadPath(
        "pathBlueLeft", new PathConstraints(2.5, 2.5));

    public PathPlannerTrajectory trajBlueLeftReturn = PathPlanner.loadPath(
        "pathBlueLeftReturn", new PathConstraints(2.2, 2.2));
        
    public PathPlannerTrajectory trajBlueRightReturn = PathPlanner.loadPath(
        "pathBlueRightReturn", new PathConstraints(2.2, 2.2));
    
    public PathPlannerTrajectory trajBlueMidBal = PathPlanner.loadPath(
            "pathBlueMidBal", new PathConstraints(1, 1));
    

    //Creates a path using the robot's initial position (from sds) and the desired position (given by vision)
    public PathPlannerTrajectory getPathUsingVision(Translation2d end_pose, Double end_heading, Double end_rotation){
        return PathPlanner.generatePath(
            new PathConstraints(4,3),
            new PathPoint(new Translation2d(sds.getPose().getX(), sds.getPose().getY()), //Creates a point (starting_position(x,y) (from sds), starting_direction(assumed 0), starting_rotation (from sds))
                Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(sds.getPose().getRotation().getDegrees())),
            new PathPoint(end_pose, Rotation2d.fromDegrees(end_heading), Rotation2d.fromDegrees(end_rotation)) //Creates a point using the given values. These values will be determined by vision
            );
    }
    
    //PID controllers for position and rotation (position is used for both x and y)
    PIDController positionControllerX = new PIDController(1.25, 0, 0);
    PIDController positionControllerY = new PIDController(1.25, 0, 0);
    PIDController thetaController = new PIDController(1.0, 0, 0);


    //This method will be called once during the beginning of autonomous
    public AutoGenerator(SwerveDrive m_sds, ArmSubsystem arm, IntakeSystem m_intake) {
        //defining the SwerveDrive used by this class as the given SwerveDrive instance
        sds = m_sds;
        intake=m_intake;

        shootCloseHighInitial=new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseHigh = new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseLow = new ShootCubeAuto(intake, closeLowSpeed);
        shootFarHigh = new ShootCubeAuto(intake, farHighSpeed);
        shootFarLow = new ShootCubeAuto(intake, farLowSpeed);
        shootFarLowEnd = new ShootCubeAuto(intake, farLowSpeed);
        shootCloseMidEnd=new ShootCubeAuto(intake, closeLowSpeed);
        balance = new autobalancer2(sds);

        //defining the CubeIntake and Stow commands used by this class by using the given ArmSubsystem and IntakeSystem
        intakeCube = new CubeIntakeAuto(arm, intake);
        cancelIntakeCube=new InstantCommand(() -> intakeCube.cancel());

        stowArm = new Stow(arm, intake);
        injectCube = new InstantCommand(() -> intake.injectCube(0) );
        endShoot = new InstantCommand(() -> intake.StopMotors());


        //When 360 degrees is exceeded, the rotation will loop back to 1 (no going over 360 degrees or 2*PI radians)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        //Putting Position PID values in the SmartDashboard
        SmartDashboard.putNumber("Position_kP", positionControllerX.getP());
        SmartDashboard.putNumber("Position_kI", positionControllerX.getI());
        SmartDashboard.putNumber("Position_kD", positionControllerX.getD());
        
        //Putting Rotation PID values in the SmartDashboard
        SmartDashboard.putNumber("Rotation_kP", thetaController.getP());
        SmartDashboard.putNumber("Rotation_kI", thetaController.getI());
        SmartDashboard.putNumber("Rotation_kD", thetaController.getD());



        //Putting default values into the Smartdashboard for everything relating to auto paths
        SmartDashboard.putBoolean("Intake_Is_On", false);
        SmartDashboard.putNumber("Path_position", 0.00);
        SmartDashboard.putString("Path3_position", "none");
        
        //These events are used in all or multiple autonomous paths
        eventMap.put("intake_cube", intakeCube);
        eventMap.put("cancel_intake_cube", cancelIntakeCube);
        eventMap.put("stow", stowArm);

        eventMap.put("shoot_close_high", shootCloseHigh);

        eventMap.put("shoot_close_low", shootCloseLow);

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
    public PPSwerveControllerCommand buildSwerveControlCommand(PathPlannerTrajectory retrievedPath) {
        return new PPSwerveControllerCommand(
            retrievedPath, //the given path, which will be run
            sds::getPose,
            SwerveDrive.m_kinematics, 
            positionControllerX, //x PID controller
            positionControllerY, //y PID controller
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
    public void updatePID(){
        positionControllerX.setPID(SmartDashboard.getNumber
        ("Position_kP", positionControllerX.getP()),
        SmartDashboard.getNumber("Position_kI", positionControllerX.getI()),
        SmartDashboard.getNumber("Position_kD", positionControllerX.getD()));


        positionControllerY.setPID(SmartDashboard.getNumber
        ("Position_kP", positionControllerX.getP()),
        SmartDashboard.getNumber("Position_kI", positionControllerX.getI()),
        SmartDashboard.getNumber("Position_kD", positionControllerX.getD()));

        
        thetaController.setPID(SmartDashboard.getNumber(
            "Rotation_kP", thetaController.getP()),SmartDashboard.getNumber(
                "Rotation_kI", thetaController.getI()),SmartDashboard.getNumber
                ("Rotation_kD", thetaController.getD()));
    }

    

    //This is a list commands to run during autonomous if testPath1 is being run
    public SequentialCommandGroup testAutoCommand1() {
        return new SequentialCommandGroup(
            new InstantCommand( () -> sds.resetOdometry(trajTestPath1.getInitialHolonomicPose())),
            followEventBuilder(trajTestPath1),
//            balance.asProxy(),
            new InstantCommand( () -> sds.allStop())
        );
    }

    //This is a list of commands to run during autonomous if testPath2 is being run


    public SequentialCommandGroup autoCommandRedRightBal(){
        return new SequentialCommandGroup(
            shootCloseHighInitial.asProxy(),
            new InstantCommand( () -> sds.resetOdometry(trajRedRightBal.getInitialHolonomicPose())),            
            followEventBuilder(trajRedRightBal),
            balance.asProxy(),
            new InstantCommand( () -> sds.allStop())
        );
    }


    public SequentialCommandGroup autoCommandRedLeftBal(){
        return new SequentialCommandGroup(
            shootCloseHighInitial.asProxy(),
            new InstantCommand( () -> sds.resetOdometry(trajRedLeftBal.getInitialHolonomicPose())),            
            followEventBuilder(trajRedLeftBal),
            balance.asProxy(),
            new InstantCommand( () -> sds.allStop())
        );
    }


    public SequentialCommandGroup autoCommandRedRight(){
        return new SequentialCommandGroup(
            shootCloseHighInitial.asProxy(),
            new InstantCommand( () -> sds.resetOdometry(trajRedRight.getInitialHolonomicPose())),            
            followEventBuilder(trajRedRight),
            new InstantCommand( () -> sds.allStop())
        );
    }


    public SequentialCommandGroup autoCommandRedLeft(){
        return new SequentialCommandGroup(
            shootCloseHighInitial.asProxy(),
            new InstantCommand( () -> sds.resetOdometry(trajRedLeft.getInitialHolonomicPose())),            
            followEventBuilder(trajRedLeft),
            new InstantCommand( () -> sds.allStop())
        );
    }

    public SequentialCommandGroup autoCommandBlueRightBal(){
        return new SequentialCommandGroup(
            shootCloseHighInitial.asProxy(),
            new InstantCommand( () -> sds.resetOdometry(trajBlueRightBal.getInitialHolonomicPose())),            
            followEventBuilder(trajBlueRightBal),
            balance.asProxy(),
            new InstantCommand( () -> sds.allStop())
        );
    }

    public SequentialCommandGroup autoCommandBlueLeftBal(){
        return new SequentialCommandGroup(
            shootCloseHighInitial.asProxy(),
            new InstantCommand( () -> sds.resetOdometry(trajBlueLeftBal.getInitialHolonomicPose())),            
            followEventBuilder(trajBlueLeftBal),
            balance.asProxy(),
            new InstantCommand( () -> sds.allStop())
        );
    }

    public SequentialCommandGroup autoCommandBlueRight(){
        return new SequentialCommandGroup(
            shootCloseHighInitial.asProxy(),
            new InstantCommand( () -> sds.resetOdometry(trajBlueRight.getInitialHolonomicPose())),            
            followEventBuilder(trajBlueRight),
            new InstantCommand( () -> sds.allStop())
        );
    }

    public SequentialCommandGroup autoCommandBlueLeft(){
        return new SequentialCommandGroup(
            shootCloseHighInitial.asProxy(),
            new InstantCommand( () -> sds.resetOdometry(trajBlueLeft.getInitialHolonomicPose())),            
            followEventBuilder(trajBlueLeft),
            new InstantCommand( () -> sds.allStop())
        );
    }



    public SequentialCommandGroup autoShootHigh(){
        return new SequentialCommandGroup(
            shootCloseHighInitial.asProxy(),
            new InstantCommand( () -> sds.allStop())
        );
    }

    public SequentialCommandGroup autoBlueLeftStay(){
        return new SequentialCommandGroup(
            shootCloseHighInitial.asProxy(),
            new InstantCommand( () -> sds.resetOdometry(trajBlueLeftReturn.getInitialHolonomicPose())),            
            followEventBuilder(trajBlueLeftReturn),
            shootCloseMidEnd.asProxy(),
            new InstantCommand( () -> sds.allStop())
            
        );
    }

    public SequentialCommandGroup autoBlueRightStay(){
        return new SequentialCommandGroup(
            shootCloseHighInitial.asProxy(),
            new InstantCommand( () -> sds.resetOdometry(trajBlueRightReturn.getInitialHolonomicPose())),            
            followEventBuilder(trajBlueRightReturn),
            shootCloseMidEnd.asProxy(),
            new InstantCommand( () -> sds.allStop())
            
        );}

        public SequentialCommandGroup autoBalanceFromMiddle(){
            return new SequentialCommandGroup(
                shootCloseHighInitial.asProxy(),
                new InstantCommand( () -> sds.resetOdometry(trajBlueMidBal.getInitialHolonomicPose())),            
                followEventBuilder(trajBlueMidBal),
                balance.asProxy(),
                new InstantCommand( () -> sds.allStop())
                
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

