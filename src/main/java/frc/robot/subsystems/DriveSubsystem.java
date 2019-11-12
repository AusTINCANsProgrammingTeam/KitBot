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
    private CANSparkMax mRight1;
    private CANSparkMax mRight2;
    private CANPIDController l_pidController;
    private CANPIDController r_pidController;
    private CANEncoder l_encoder;
    private CANEncoder r_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    public DriveSubsystem(){   
        mLeft1 = new CANSparkMax(1, MotorType.kBrushless);
        mLeft2 = new CANSparkMax(2, MotorType.kBrushless);
        mRight1 = new CANSparkMax(3, MotorType.kBrushless);
        mRight2 = new CANSparkMax(4, MotorType.kBrushless);
        mLeft1.restoreFactoryDefaults();
        mLeft2.restoreFactoryDefaults();
        mRight1.restoreFactoryDefaults();
        mRight2.restoreFactoryDefaults();
        mLeft1.enableVoltageCompensation(12);
        mLeft2.enableVoltageCompensation(12);
        mRight1.enableVoltageCompensation(12);
        mRight2.enableVoltageCompensation(12);
        mLeft1.setIdleMode(IdleMode.kBrake);
        mLeft2.setIdleMode(IdleMode.kBrake);
        mRight1.setIdleMode(IdleMode.kBrake);
        mRight2.setIdleMode(IdleMode.kBrake);
        mLeft2.follow(mLeft1);
        mRight2.follow(mRight1);

        /**
         * In order to use PID functionality for a controller, a CANPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        l_pidController = mLeft1.getPIDController();
        r_pidController = mRight1.getPIDController();

        // Encoder object created to display position values
        l_encoder = mLeft1.getEncoder();
        r_encoder = mLeft1.getEncoder();

        // PID coefficients
        kP = 0.0006; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        l_pidController.setP(kP);
        l_pidController.setI(kI);
        l_pidController.setD(kD);
        l_pidController.setIZone(kIz);
        l_pidController.setFF(kFF);
        l_pidController.setOutputRange(kMinOutput, kMaxOutput);
        r_pidController.setP(kP);
        r_pidController.setI(kI);
        r_pidController.setD(kD);
        r_pidController.setIZone(kIz);
        r_pidController.setFF(kFF);
        r_pidController.setOutputRange(kMinOutput, kMaxOutput);
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
        // if((p != kP)) { m_pidController.setP(p); kP = p; }
        // if((i != kI)) { m_pidController.setI(i); kI = i; }
        // if((d != kD)) { m_pidController.setD(d); kD = d; }
        // if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        // if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        // if((max != kMaxOutput) || (min != kMinOutput)) 
        { 
            // m_pidController.setOutputRange(min, max); 
            // kMinOutput = min;
            // kMaxOutput = max; 
        }
    }

    public void setLeftPidVelocitySetpoint(double setpoint)
    {
        l_pidController.setReference(setpoint, ControlType.kVelocity);
    }

    public void setRightPidVelocitySetpoint(double setpoint)
    {
        r_pidController.setReference(setpoint, ControlType.kVelocity);
    }

    public double leftVelocity()
    {
        return l_encoder.getVelocity();
    }

    public double rightVelocity()
    {
        return r_encoder.getVelocity();
    }

    @Override
  public void initDefaultCommand() {
      
  }
}