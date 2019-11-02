package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.RobotMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.logging.*;

public class DriveSubsystem extends Subsystem{
    private DifferentialDrive differentialDrive;
    private CANSparkMax mLeftSparkMax1;
    private CANSparkMax mLeftSparkMax2;
    private CANSparkMax mRightSparkMax1;
    private CANSparkMax mRightSparkMax2;

    private SparkMaxGroup mLeftSparkMaxGroup;
    private SparkMaxGroup mRightSparkMaxGroup;
    private boolean mClosedLoopControl = false;

    public DriveSubsystem()
    {   
        mLeftSparkMax1 = new CANSparkMax(RobotMap.LEFTMOTOR_1, MotorType.kBrushless);
        mLeftSparkMax2 = new CANSparkMax(RobotMap.LEFTMOTOR_2, MotorType.kBrushless);
        mRightSparkMax1 = new CANSparkMax(RobotMap.RIGHTMOTOR_1, MotorType.kBrushless);
        mRightSparkMax2 = new CANSparkMax(RobotMap.RIGHTMOTOR_2, MotorType.kBrushless);
        mLeftSparkMaxGroup = new SparkMaxGroup(Constants.kGains_Velocity, mLeftSparkMax1, mLeftSparkMax2);
        mRightSparkMaxGroup =  new SparkMaxGroup(Constants.kGains_Velocity, mRightSparkMax1, mRightSparkMax2);

        differentialDrive = new DifferentialDrive(
            mLeftSparkMaxGroup, mRightSparkMaxGroup
            );
        differentialDrive.setSafetyEnabled(false);
    }

    /**
     * Set if the drive base should behave in closed loop control or
     * open loop control
     * Basically if pid is running or not
     * @param closedLoopControl
     */
    public void setClosedLoopControl(boolean closedLoopControl)
    {
        mClosedLoopControl = closedLoopControl;
        mLeftSparkMaxGroup.setBrakeMode(!mClosedLoopControl);
        mRightSparkMaxGroup.setBrakeMode(!mClosedLoopControl);
    }

    /**
     * Arcade drive the robot
     * @param velocity
     * @param heading
     */
    public void arcadeDrive(double velocity, double heading)
    {
        //Check that we are in open loop control be attempting to drive the robot
        if (!mClosedLoopControl)
        {
            this.differentialDrive.arcadeDrive(velocity, heading * .70, true);
        }
    }

    /**
     * Set the set point of the left side of the drive base
     * @param setPoint Command to set
     */
    public void setLeftVelocity(double setPoint)
    {
        //Check that we are in closed loop control be attempting to set the set point
        if (mClosedLoopControl)
        {
            mLeftSparkMaxGroup.setVelocity(setPoint);
        }
    }

    /**
     * Set the setpoint of the right side of the drive base
     * @param setPoint
     */
    public void setRightVelocity(double setPoint)
    {
        //Check that we are in closed loop control be attempting to set the set point
        if (mClosedLoopControl)
        {
            mLeftSparkMaxGroup.setVelocity(setPoint);
        }
    }

    /**
     * Get the velocity of the left side of the drive base
     * @return Velocity of the left side
     */
    public double leftVelocity()
    {
        return mLeftSparkMaxGroup.getVelocity();
    }

    /**
     * get the velocity of the right side of the drive base
     * @return Velocity of the right side
     */
    public double rightVelocity()
    {
        return mRightSparkMaxGroup.getVelocity();
    }

    @Override
    public void initDefaultCommand()
    {
      
    }
}