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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.logging.*;

public class DriveSubsystem extends Subsystem{
    private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());

    private CANSparkMax mLeft1;
    private CANSparkMax mLeft2;
    private CANPIDController m_pidController;
    private CANEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    public DriveSubsystem(){   
        mLeft1 = new CANSparkMax(1, MotorType.kBrushless);
        mLeft2 = new CANSparkMax(2, MotorType.kBrushless);
        mLeft1.restoreFactoryDefaults();
        mLeft2.restoreFactoryDefaults();
        //mLeft1.enableVoltageCompensation(12);
        //mLeft2.enableVoltageCompensation(12);
        mLeft1.setIdleMode(IdleMode.kBrake);
        mLeft2.setIdleMode(IdleMode.kBrake);
        mLeft2.follow(mLeft1);

        /**
         * In order to use PID functionality for a controller, a CANPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = mLeft1.getPIDController();

        // Encoder object created to display position values
        m_encoder = mLeft1.getEncoder();

        // PID coefficients
        kP = 0.0006; 
        kI = 0.0000005;
        kD = 0.00005; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

         // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
    }

    public void arcadeDrive(double velocity, double heading) {
        //this.differentialDrive.arcadeDrive(velocity, heading * .70, true);
    }

    public void updatePID()
    {
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput))         
        { 
            m_pidController.setOutputRange(min, max); 
            kMinOutput = min;
            kMaxOutput = max; 
        }
    }

    public void setLeftVelocitySetpoint(double setpoint)
    {
        m_pidController.setReference(setpoint, ControlType.kVelocity);
    }

    public double leftVelocity()
    {
        return m_encoder.getVelocity();
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