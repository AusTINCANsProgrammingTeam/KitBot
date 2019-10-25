package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import java.util.logging.Logger;

public class SparkMaxGroup implements SpeedController {
    private static final Logger LOGGER = Logger.getLogger(SparkMaxGroup.class.getName());
    private CANSparkMax master;
    private static CANPIDController m_pidController;
    private static CANPIDController s_pidController;
    private CANEncoder m_encoder;
    private CANSparkMax [] slaves;
    private double kP;
    private double kI;
    private double kD;
    public static int maxRPM = 5600;

    /**
     *
     * Creates a new TalonSRXGroup from a set of WPI_TalonSRX instances. The first instance is used as the master.
     * @param master the master motor. All other motors will follow this one.
     * @param slaves the slave motors. Follow the master motor.
     */
    public SparkMaxGroup(CANSparkMax master, CANSparkMax... slaves) {
        
        this.slaves = slaves;
        this.master = master;
        //m_encoder = master.getEncoder();
        //master.restoreFactoryDefaults();
       // m_pidController = master.getPIDController();
        for(CANSparkMax slave : slaves) {
            slave.restoreFactoryDefaults();
            slave.follow(master);
        }
        
        //m_pidController.setP(Constants.kGains_Velocity.kP);
        //m_pidController.setI(Constants.kGains_Velocity.kI);
        //m_pidController.setD(Constants.kGains_Velocity.kD);
        //m_pidController.setIZone(Constants.kGains_Velocity.kD);
        //m_pidController.setFF(Constants.kGains_Velocity.kD);
        //m_pidController.setOutputRange(Constants.kGains_Velocity.kMinOutput, Constants.kGains_Velocity.kMaxOutput);
    }

    /**
     * Set the output to the value calculated by PIDController.
     * @param output the value calculated by PIDController.
     */
    @Override
    public void pidWrite(double output) {
    }
    
    public static int getRpm() {
        return maxRPM;
    }

    /**
     * Common interface for setting the speed of a speed controller.
     * @param speed the speed to set. Value should be between -1.0 and 1.0.
     */
    @Override
    public void set(double speed) {
        master.set(speed);
    }


    /**
     * Common interface for getting the current set speed of the speed controller.
     * @return the current set speed. Value is between -1.0 and 1.0.
     */
    @Override
    public double get() {
        return master.get();
    }

    /**
     * Common interface for inverting direction of a speed controller.
     * @param isInverted the state of inversion; true is inverted.
     */
    @Override
    public void setInverted(boolean isInverted) {
        master.setInverted(isInverted);
        for(CANSparkMax slave : slaves) {
            slave.setInverted(isInverted);
        }
    }

    /**
     * Common interface for returning if a speed controller is in the inverted state or not.
     * @return the state of the inversion; true is inverted.
     */
    @Override
    public boolean getInverted() {
        return master.getInverted();
    }

    /**
     * Disable the speed controller.
     */
    @Override
    public void disable() {
        master.disable();
        for(CANSparkMax slave : slaves) {
            slave.disable();
        }
    }

    /**
     * Stops motor movement. Motor can be moved again by calling set without having to re-enable the motor.
     */
    @Override
    public void stopMotor() {
        master.stopMotor();
        for(CANSparkMax slave : slaves) {
            slave.stopMotor();
        }
    }
}
