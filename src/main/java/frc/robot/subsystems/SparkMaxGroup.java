package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.Robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import java.util.logging.Logger;

public class SparkMaxGroup implements SpeedController {
    private CANSparkMax mMaster;
    private static CANPIDController mPidController;
    private CANEncoder mEncoder;
    private CANSparkMax [] mSlaves;

    /**
     *
     * Creates a new SparkmaxGroup from a set of Spark maxes instances. The first instance is used as the master.
     * @param master the master motor. All other motors will follow this one.
     * @param slaves the slave motors. Follow the master motor.
     */
    public SparkMaxGroup(Gains pidGains, CANSparkMax master, CANSparkMax... slaves) 
    {
        //Assign the array of incoming slaves to this class's slaves.        
        mSlaves = slaves;
        //Assign the incoming master to this class's Master
        mMaster = master;
        //Create an encoder for the spark max group. The Motors are joined mechanically
        //to the gearbox so they spin in sync. Which means we only need one encoder.
        mEncoder = master.getEncoder();

        //Reset the factory settings to clear any incorrect configuration values
        master.restoreFactoryDefaults();

        //Create the single pid controller that all spark maxes will use.
        mPidController = master.getPIDController();

        //Reset each configuration on the slaves, and set them to follow the master
        for(CANSparkMax slave : slaves) {
            slave.restoreFactoryDefaults();
            slave.follow(master);
        }
        
        //Set the gains for the pid controller
        mPidController.setP(pidGains.kP);
        mPidController.setI(pidGains.kI);
        mPidController.setD(pidGains.kD);
        mPidController.setIZone(pidGains.kD);
        mPidController.setFF(pidGains.kD);
        mPidController.setOutputRange(pidGains.kMinOutput, pidGains.kMaxOutput);
        
        setBrakeMode(false);
    }

    /**
     *Set the velocity setpoint of the pid controller 
     */
    public void setVelocity(double setPoint)
    {
        mPidController.setReference(setPoint, ControlType.kVelocity);
    }

    public double getVelocity()
    {
        return mEncoder.getVelocity();
    }

    /**
     * Set all of the spark maxes to the specified idle mode
     * @param brakeMode if the spark max group should be set to brake mode
     */
    public void setBrakeMode(boolean brakeMode)
    {
        //if (brakeMode)
        //{
        //  mode = IdleMode.kCoast;
        //}
        //else
        //{
        //  mode = IdleMode.kBrake;
        //}
        // ? Is the ternary opperator the below statement is equivlent to the above if else
        // If the condition is true it will assign the value before the semi colon to mode.
        // If the condition is false it will assign the value after the semi colon to mode.
        //              Conditon to check
        //                          if (true)         if (false)
        IdleMode mode = brakeMode ? IdleMode.kBrake : IdleMode.kCoast;
        //Set the master and slave to specified mode
        mMaster.setIdleMode(mode);
        for (CANSparkMax slave : mSlaves)
        {
            slave.setIdleMode(mode);
        }
    }

    /**
     * Set the output to the value calculated by PIDController.
     * @param output the value calculated by PIDController.
     */
    @Override
    public void pidWrite(double output) {
        mMaster.set(output);
    }

    /**
     * Common interface for setting the speed of a speed controller.
     * @param speed the speed to set. Value should be between -1.0 and 1.0.
     */
    @Override
    public void set(double speed) {
        mMaster.set(speed);
    }


    /**
     * Common interface for getting the current set speed of the speed controller.
     * @return the current set speed. Value is between -1.0 and 1.0.
     */
    @Override
    public double get() {
        return mMaster.get();
    }

    /**
     * Common interface for inverting direction of a speed controller.
     * @param isInverted the state of inversion; true is inverted.
     */
    @Override
    public void setInverted(boolean isInverted) {
        mMaster.setInverted(isInverted);
        for(CANSparkMax slave : mSlaves) {
            slave.setInverted(isInverted);
        }
    }

    /**
     * Common interface for returning if a speed controller is in the inverted state or not.
     * @return the state of the inversion; true is inverted.
     */
    @Override
    public boolean getInverted() {
        return mMaster.getInverted();
    }

    /**
     * Disable the speed controller.
     */
    @Override
    public void disable() {
        mMaster.disable();
        for(CANSparkMax slave : mSlaves) {
            slave.disable();
        }
    }

    /**
     * Stops motor movement. Motor can be moved again by calling set without having to re-enable the motor.
     */
    @Override
    public void stopMotor() {
        mMaster.stopMotor();
        for(CANSparkMax slave : mSlaves) {
            slave.stopMotor();
        }
    }
}
