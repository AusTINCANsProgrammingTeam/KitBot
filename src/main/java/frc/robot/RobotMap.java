/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
   public final static int LEFTMOTOR_1 = 1;
   public final static int LEFTMOTOR_2 = 2;
   //public final static int LEFTMOTOR_3 = 7;
   public final static int RIGHTMOTOR_1 = 3;
   public final static int RIGHTMOTOR_2 = 4;
   //public final static int RIGHTMOTOR_3 = 3;
   public final static int joystick = 0;
}
