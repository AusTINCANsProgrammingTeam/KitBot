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
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.logging.*;

public class DriveSubsystem extends Subsystem{
    private DifferentialDrive differentialDrive;
    private static CANSparkMax m_leftmotor1;
    private static CANSparkMax m_leftmotor2;
    private static CANSparkMax m_rightmotor1;
    private static CANSparkMax m_rightmotor2;
    private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());


    public DriveSubsystem(){   
        m_leftmotor1 = new CANSparkMax(RobotMap.LEFTMOTOR_1, MotorType.kBrushless);
        m_leftmotor2 = new CANSparkMax(RobotMap.LEFTMOTOR_2, MotorType.kBrushless);
        m_rightmotor1 = new CANSparkMax(RobotMap.RIGHTMOTOR_1, MotorType.kBrushless);
        m_rightmotor2 = new CANSparkMax(RobotMap.RIGHTMOTOR_2, MotorType.kBrushless);
        m_leftmotor1.restoreFactoryDefaults();
        //m_leftmotor2.restoreFactoryDefaults();
        m_rightmotor1.restoreFactoryDefaults();
        //m_rightmotor2.restoreFactoryDefaults();
        differentialDrive = new DifferentialDrive(
            new SparkMaxGroup(m_leftmotor1, m_leftmotor2),
            new SparkMaxGroup(m_rightmotor1, m_rightmotor2)
            );
        differentialDrive.setSafetyEnabled(false);
    }

    public void arcadeDrive(double velocity, double heading) {
        this.differentialDrive.arcadeDrive(velocity, heading * .70, true);
    }

    @Override
  public void initDefaultCommand() {
      
  }
}