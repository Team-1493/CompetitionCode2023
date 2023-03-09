package frc.robot.commands;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Sensors.Pigeon;
import java.lang.Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class autobalancer { 
    public SwerveDrive sds;
    public Pigeon getheadingDegreesPigeon;
    public double MaxPitchValue = 3;


    public autobalancer(SwerveDrive m_sds) {
        sds = m_sds;
    }

    public void getandsetheading() {
        double desiredHeading=0;
        desiredHeading = Math.round(sds.heading/(Math.PI/2));
        double heading = desiredHeading * (180/Math.PI);
        
        sds.setHeading(heading);
        if(heading<30 && heading> -30){
            System.out.println("heading"+heading);
            //use pitch
            double Pitch = sds.pitch;
            System.out.println("pitch = "+ Pitch);
            SmartDashboard.putNumber("using pitch", Pitch);
            if (Pitch > MaxPitchValue){
                double xpos = sds.getPose().getX();
                while (Pitch > MaxPitchValue){
                    System.out.print("xpos:"+xpos);
                    Pitch = Pigeon.Instance.getPitch();
                    System.out.println("pitch"+Pitch);
                    sds.setMotors(-0.25, 0, 0);
                    xpos = sds.getPose().getX();

                }
            }
            if (Pitch < -MaxPitchValue){
                Double xpos = sds.getPose().getX();
                System.out.println("Init xpos="+xpos);
                while (Pitch < -MaxPitchValue){
                    Pitch = Pigeon.Instance.getPitch();
                    // System.out.println("pitch"+pitch);
                    xpos = sds.getPose().getX();
                    // System.out.println("xpos="+xpos);
                    sds.setMotors(0.25, 0, 0);
                    Double Desiredxpos = sds.getPose().getX();
                    // System.out.println("desired xpos = " +Desiredxpos);
                
                }
            }
        } 
        if(heading>100 || heading<-100){
            //use pitch
            Double pitch = sds.pitch;
            SmartDashboard.putNumber("using pitch", pitch);
            if (pitch > MaxPitchValue){
                Double xpos = sds.getPose().getX();
                while (pitch > MaxPitchValue){
                    System.out.println("heading:"+heading);
                    pitch = Pigeon.Instance.getPitch();
                    System.out.println("pitch"+pitch);
                    sds.setMotors(0.25, 0, 0);
                    xpos = sds.getPose().getX();
                    System.out.println("xpos = "+xpos);

                }
            }
                if (pitch < -MaxPitchValue){
                    Double xpos = sds.getPose().getX();
                    while (pitch < -MaxPitchValue){
                        pitch = Pigeon.Instance.getPitch();
                        sds.setMotors(-0.25, 0, 0);
                        xpos = sds.getPose().getX(); 
                        System.out.println("xpos = "+xpos);
                    }
                }
        }
    }
}