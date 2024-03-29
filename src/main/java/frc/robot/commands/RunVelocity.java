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

public class RunVelocity extends Command {
  private double timeToRun;
  private double velocity;
  private Timer timer;

  public RunVelocity(double velRPM, int timeToRunMs) {
    timeToRun = timeToRunMs;
    velocity = velRPM;
    timer = new Timer();
    requires(Robot.mDriveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer.reset();
    timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.mDriveSubsystem.setLeftVelocitySetpoint(velocity);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return timer.get() >= timeToRun;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Zero the velocity
    Robot.mDriveSubsystem.setLeftVelocitySetpoint(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
