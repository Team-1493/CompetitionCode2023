package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

// This is more of a utility subsystem than an actual subsystem

public class Stick extends SubsystemBase {
  public Joystick joy0;
  public POVButton pov0;
  public POVButton pov90;
  public POVButton pov180;
  public POVButton pov270;
  
  public Stick(int port) {
    joy0=new Joystick(port);
    pov0=new POVButton(joy0, 0);
    pov90=new POVButton(joy0, 90);
    pov180=new POVButton(joy0, 180);
    pov270=new POVButton(joy0, 270);
  }

  // read the joystick and return an array containing:
  // [0]  squared magnitude in for/back direction (for is pos)
  // [1]  squared magnitude in left/right direction (right is pos)
  // [2]  provides rotational speed
  // all values are -1 to 1
  public double[] readStick(){
    double[] stickValues = new double[3];
    double direction = joy0.getDirectionDegrees();
    double mag=joy0.getMagnitude();
    if (Math.abs(mag)<0.04) mag=0;
 
    double omega;
    omega = joy0.getRawAxis(4);
    if (Math.abs(omega)<0.04) omega=0;
   omega = 4*omega;
    
    stickValues[0]= mag*mag*Math.cos(direction*Math.PI/180);
    stickValues[1]=mag*mag*Math.sin(direction*Math.PI/180);
    stickValues[2]=omega;
    return stickValues;
  }

//   returns  a joystick button  (Not the value of the button!)
  public JoystickButton getButton(int buttonNumber){
    return new JoystickButton(joy0, buttonNumber);
  }

  public BooleanSupplier mapStick(int axis){
    return new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return (joy0.getRawAxis(axis)>0.5);
      }
      
    };
  }


}
