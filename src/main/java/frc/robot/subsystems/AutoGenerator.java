package frc.robot.subsystems;
// comment added from computer 12 for github test
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CubeIntake;
import frc.robot.commands.CubeIntakeAuto;
import frc.robot.commands.ShootCubeAuto;
import frc.robot.commands.Stow;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;


public class AutoGenerator extends SubsystemBase{
    //Defining the SwerveDrive subsystem used by the AutoGenerator in a placeholder variable
    private SwerveDrive sds;   
    
    //Defining the CubeIntake and Stow commands used by the AutoGenerator in placeholder variables
    private CubeIntakeAuto intakeCube;
    private Stow stowArm;

    private ShootCubeAuto shootCloseHigh;
    private ShootCubeAuto shootCloseLow;
    private ShootCubeAuto shootFarHigh;
    private ShootCubeAuto shootFarLow;


    private int closeHighSpeed = 3; //placeholder
    private int closeLowSpeed = 2; //placeholder
    private int farHighSpeed = 5; //placeholder
    private int farLowSpeed = 4; //placeholder

    //Defining a HashMap called eventMap, which will store all events that can run during auto
    private HashMap<String, Command> eventMap = new HashMap<>();
    
    
    //Loading all autonomous paths and storing them in variables
//    public PathPlannerTrajectory testPath1 = PathPlanner.loadPath("testPath1", new PathConstraints(4, 3));
    public PathPlannerTrajectory testPath1 = PathPlanner.loadPath("path2b", new PathConstraints(2, 2));
    public PathPlannerTrajectory testPath2 = PathPlanner.loadPath("testPath2", new PathConstraints(2, 2));
    public PathPlannerTrajectory testPath3 = PathPlanner.loadPath("testPath3", new PathConstraints(4, 3)); 

    public PathPlannerTrajectory path1 = PathPlanner.loadPath("path1", new PathConstraints(2, 2));

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
    PIDController positionController = new PIDController(.0025, 0, 0);
    PIDController thetaController = new PIDController(1.0, 0, 0);


    //This method will be called once during the beginning of autonomous
    public AutoGenerator(SwerveDrive m_sds, ArmSubsystem arm, IntakeSystem intake) {
        //defining the SwerveDrive used by this class as the given SwerveDrive instance
        sds = m_sds;

        //defining the CubeIntake and Stow commands used by this class by using the given ArmSubsystem and IntakeSystem
        intakeCube = new CubeIntakeAuto(arm, intake);
        stowArm = new Stow(arm, intake);

        shootCloseHigh = new ShootCubeAuto(intake, closeHighSpeed);
        shootCloseLow = new ShootCubeAuto(intake, closeLowSpeed);

        shootFarHigh = new ShootCubeAuto(intake, farHighSpeed);
        shootFarLow = new ShootCubeAuto(intake, farLowSpeed);

        //When 360 degrees is exceeded, the rotation will loop back to 1 (no going over 360 degrees or 2*PI radians)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        //Putting Position PID values in the SmartDashboard
        SmartDashboard.putNumber("Position_kP", positionController.getP());
        SmartDashboard.putNumber("Position_kI", positionController.getI());
        SmartDashboard.putNumber("Position_kD", positionController.getD());
        
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
        eventMap.put("stow", stowArm);

        eventMap.put("shoot_close_high", shootCloseHigh);

        eventMap.put("shoot_close_low", shootCloseLow);

        eventMap.put("shoot_far_high", shootFarHigh);
        eventMap.put("shoot_far_low", shootFarLow);

        

        
        
        
        //The multi-line comment below is for testing if needed
     /* double timeEnd = testPath1.getEndState().timeSeconds;
        double t = 0;
        while  (t<timeEnd){
            State state = testPath1.sample(t);
            System.out.println(state.poseMeters.getX()+",  "+state.poseMeters.getY()+", "+state.poseMeters.getRotation().getDegrees()+", "+", "+state.velocityMetersPerSecond);
            t=t+0.1;
        };
       System.out.println(testPath1.getMarkers().get(1));
       System.out.println(testPath1.getMarkers().get(2));
       System.out.println(testPath1.getMarkers().get(3)); */
    }
    


    //Builds and returns a PPSwerveControllerCommand for the given path
    public PPSwerveControllerCommand buildSwerveControlCommand(PathPlannerTrajectory retrievedPath) {
        return new PPSwerveControllerCommand(
            retrievedPath, //the given path, which will be run
            sds::getPose,
            SwerveDrive.m_kinematics, 
            positionController, //x PID controller
            positionController, //y PID controller
            thetaController, //rotation PID controller
            sds::setModuleStates, 
            true, //if the robot is on the red alliance, the path will be reflected
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
        positionController.setPID(SmartDashboard.getNumber("Position_kP", positionController.getP()),SmartDashboard.getNumber("Position_kI", positionController.getI()),SmartDashboard.getNumber("Position_kD", positionController.getD()));
        thetaController.setPID(SmartDashboard.getNumber("Rotation_kP", positionController.getP()),SmartDashboard.getNumber("Rotation_kI", positionController.getI()),SmartDashboard.getNumber("Rotation_kD", positionController.getD()));
    }

    

    //This is a list of commands to run during autonomous if testPath1 is being run
    public SequentialCommandGroup testAutoCommand1() {
        return new SequentialCommandGroup(
            shootCloseHigh,
            new InstantCommand( () -> sds.resetOdometry(testPath1.getInitialHolonomicPose())),
            followEventBuilder(path1),
            shootFarLow,
            new InstantCommand( () -> sds.allStop())
        );
    }

    //This is a list of commands to run during autonomous if testPath2 is being run
    public SequentialCommandGroup testAutoCommand2(){
        return new SequentialCommandGroup(
            new InstantCommand( () -> sds.resetOdometry(testPath2.getInitialHolonomicPose())),
            followEventBuilder(testPath2),
            new InstantCommand( () -> sds.allStop())
        );
    }

    //This is a list of commands to run during autonomous if testPath3 is being run
    public SequentialCommandGroup testAutoCommand3(){
        return new SequentialCommandGroup(
            new InstantCommand( () -> sds.resetOdometry(testPath3.getInitialHolonomicPose())),
            followEventBuilder(testPath3),
            new InstantCommand( () -> sds.allStop())
        );
    }

    public SequentialCommandGroup autoCommand1(){
        return new SequentialCommandGroup(
            new InstantCommand( () -> sds.resetOdometry(path1.getInitialHolonomicPose())),
//            shootCloseHigh,
            followEventBuilder(path1),
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

