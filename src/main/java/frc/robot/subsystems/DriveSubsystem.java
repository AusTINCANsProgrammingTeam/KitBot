package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.logging.*;

public class DriveSubsystem extends Subsystem{
    private DifferentialDrive differentialDrive;
    private static CANSparkMax m_leftmotor1;
    private static CANSparkMax m_leftmotor2;
    private static CANSparkMax m_rightmotor1;
    private static CANSparkMax m_rightmotor2;

    private static SparkMaxGroup leftSparkMaxes;
    private static SparkMaxGroup rightSparkMaxes;
    private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());


    public DriveSubsystem(){   
        m_leftmotor1 = new CANSparkMax(RobotMap.LEFTMOTOR_1, MotorType.kBrushless);
        m_leftmotor2 = new CANSparkMax(RobotMap.LEFTMOTOR_2, MotorType.kBrushless);
        m_rightmotor1 = new CANSparkMax(RobotMap.RIGHTMOTOR_1, MotorType.kBrushless);
        m_rightmotor2 = new CANSparkMax(RobotMap.RIGHTMOTOR_2, MotorType.kBrushless);
        m_leftmotor1.restoreFactoryDefaults();
        m_rightmotor1.restoreFactoryDefaults();
        leftSparkMaxes = new SparkMaxGroup(m_leftmotor1, m_leftmotor2);
        rightSparkMaxes =  new SparkMaxGroup(m_rightmotor1, m_rightmotor2);
        differentialDrive = new DifferentialDrive(
            leftSparkMaxes, rightSparkMaxes
            );
        differentialDrive.setSafetyEnabled(false);
    }

    public void arcadeDrive(double velocity, double heading) {
        this.differentialDrive.arcadeDrive(velocity, heading * .70, true);
    }

    public void setPidVelocitySetpoint(double setpoint)
    {
        leftSparkMaxes.getpidController().setReference(.1, ControlType.kVelocity);
    }

    public double leftVelocity()
    {
        return m_leftmotor1.getEncoder().getVelocity();
    }

    public double rightVelocity()
    {
        return m_rightmotor1.getEncoder().getVelocity();
    }

    @Override
  public void initDefaultCommand() {
      
  }
}