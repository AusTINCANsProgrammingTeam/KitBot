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
    private CANSparkMax mLeftmotor1;
    private CANSparkMax mLeftmotor2;
    private CANSparkMax mRightmotor1;
    private CANSparkMax mRightmotor2;

    private CANPIDController mLeftPidController;
    private CANPIDController mRightPidController;

    private CANEncoder mLeftEncoder;
    private CANEncoder mRightEncoder;

    private SparkMaxGroup mLeftSparkMaxes;
    private SparkMaxGroup mRightSparkMaxes;
    private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());


    public DriveSubsystem(){   
        mLeftmotor1 = new CANSparkMax(RobotMap.LEFTMOTOR_1, MotorType.kBrushless);
        mLeftmotor2 = new CANSparkMax(RobotMap.LEFTMOTOR_2, MotorType.kBrushless);
        mRightmotor1 = new CANSparkMax(RobotMap.RIGHTMOTOR_1, MotorType.kBrushless);
        mRightmotor2 = new CANSparkMax(RobotMap.RIGHTMOTOR_2, MotorType.kBrushless);
        mLeftmotor1.restoreFactoryDefaults();
        mRightmotor1.restoreFactoryDefaults();
        mLeftSparkMaxes = new SparkMaxGroup(mLeftmotor1, mLeftmotor2);
        mRightSparkMaxes =  new SparkMaxGroup(mRightmotor1, mRightmotor2);

        mLeftPidController = mLeftmotor1.getPIDController();
        mLeftPidController.setP(Constants.kGains_Velocity.kP);
        mLeftPidController.setI(Constants.kGains_Velocity.kI);
        mLeftPidController.setD(Constants.kGains_Velocity.kD);
        mLeftPidController.setIZone(Constants.kGains_Velocity.kIZone);
        mLeftPidController.setFF(0);
        mLeftPidController.setOutputRange(Constants.kGains_Velocity.kMinOutput, Constants.kGains_Velocity.kMaxOutput);


        mRightPidController = mRightmotor1.getPIDController();

        mLeftEncoder = mLeftmotor1.getEncoder();
        mRightEncoder = mRightmotor1.getEncoder();

        differentialDrive = new DifferentialDrive(
            mLeftSparkMaxes, mRightSparkMaxes
            );
        differentialDrive.setSafetyEnabled(false);
    }

    public void arcadeDrive(double velocity, double heading) {
        this.differentialDrive.arcadeDrive(velocity, heading * .70, true);
    }

    public void setPidVelocitySetpoint(double setpoint)
    {
        mLeftPidController.setReference(1000, ControlType.kVelocity);
    }

    public double leftVelocity()
    {
        return mLeftEncoder.getVelocity();
    }

    public double rightVelocity()
    {
        return mRightEncoder.getVelocity();
    }

    @Override
  public void initDefaultCommand() {
      
  }
}