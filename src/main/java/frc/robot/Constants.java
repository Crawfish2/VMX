
package frc.robot;
public final class Constants
 {
    public static final int TITAN_ID        = 42; 
    public static final int MOTOR0          = 0;

    // Omni wheel radius(mm)
    public static final double wheelRadius = 50;
    // Encoder pulse per revolution
    public static final double pulsePerRevolution = 1464;
    // Gear Ratio between encoder and wheel
    public static final double gearRatio = 1/1;
    // Pulse per revolution of wheel
    public static final double wheelPulseRatio = pulsePerRevolution * gearRatio;
    // Distance per tick
    public static final double WHEEL_DIST_PER_TICK = (Math.PI * 2 * wheelRadius) / wheelPulseRatio;

}
