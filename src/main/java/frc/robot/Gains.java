package frc.robot;

public class Gains {
    public final double kP;
    public final double kI;
    public final double kD;
    public final int kIZone;
    public final int kMaxOutput;
    public final int kMinOutput;

    public Gains(double p, double i, double d,
     int iZone, int minOutput, int maxOutput)
    {
        kP = p;
        kI = i;
        kD = d;
        kIZone = iZone;
        kMaxOutput = maxOutput;
        kMinOutput = minOutput;
    }
}