/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;
import frc.robot.Robot;
import com.revrobotics.ControlType;
import java.util.logging.*;

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveCommand extends Command {
    private static final Logger LOGGER = Logger.getLogger(DriveCommand.class.getName());
    public Joystick joystick = new Joystick(RobotMap.joystick);
  public DriveCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.mDriveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      Robot.mDriveSubsystem.arcadeDrive(joystick.getRawAxis(1), joystick.getRawAxis(2));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
