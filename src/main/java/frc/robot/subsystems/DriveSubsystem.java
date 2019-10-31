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
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.logging.*;

public class DriveSubsystem extends Subsystem{
    private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());

    private static final int deviceID = 1;
    private CANSparkMax mLeft1;
    private CANSparkMax mLeft2;
    private CANPIDController m_pidController;
    private SparkMaxGroup mLeftSparkGroup;
    private CANEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    public DriveSubsystem(){   
        mLeft1 = new CANSparkMax(2, MotorType.kBrushless);
        mLeft2 = new CANSparkMax(1, MotorType.kBrushless);

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        mLeft1.restoreFactoryDefaults();
        mLeft2.restoreFactoryDefaults();
        //mLeft2.follow(mLeft1);
    
       // mLeftSparkGroup = new SparkMaxGroup(mLeft1, mLeft2);

        /**
         * In order to use PID functionality for a controller, a CANPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = mLeft1.getPIDController();

        // Encoder object created to display position values
        m_encoder = mLeft1.getEncoder();

        // PID coefficients
        kP = 0.0001; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void arcadeDrive(double velocity, double heading) {
        //this.differentialDrive.arcadeDrive(velocity, heading * .70, true);
    }

    public void setPidVelocitySetpoint(double setpoint)
    {
        //LOGGER.warning("Set point is: " + setpoint);
        //SmartDashboard.putNumber("setpoint", setpoint);
        //CANError error = mLeftPidController.setReference(setpoint, ControlType.kVelocity);
        //LOGGER.warning(error.toString());

        
        m_pidController.setReference(setpoint, ControlType.kVelocity);
    }

    public double leftVelocity()
    {
        //return mLeftEncoder.getVelocity();
        return 0.0;
    }

    public double rightVelocity()
    {
        //return mRightEncoder.getVelocity();
        return 0.0;
    }

    @Override
  public void initDefaultCommand() {
      
  }
}