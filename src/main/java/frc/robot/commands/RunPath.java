/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.List;
import java.util.logging.*;

public class RunPath extends Command {
  private List rightPath;
  private List leftPath;
  private Timer timer;
  private double timeToRun;
  private int i = 0;
  private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());

  public RunPath(List leftPath, List rightPath) {
    this.rightPath = rightPath;
    this.leftPath = leftPath;
    timeToRun = .020;
    timer = new Timer();
    requires(Robot.mDriveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // timer.reset();
    // timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.mDriveSubsystem.setLeftPidVelocitySetpoint(Double.valueOf(leftPath.get(i).toString()));
    Robot.mDriveSubsystem.setRightPidVelocitySetpoint(Double.valueOf(rightPath.get(i).toString()));
    i++;
}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // return timer.get() >= timeToRun;
    return i >= leftPath.size();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Zero the velocity
    Robot.mDriveSubsystem.setLeftPidVelocitySetpoint(0);
    Robot.mDriveSubsystem.setRightPidVelocitySetpoint(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
