package frc.robot.util;

public class DriveSpeed {
  public final double vx;
  public final double vy;
  public final double zRotation;

  public DriveSpeed() {
    vx = 0.0;
    vy = 0.0;
    zRotation = 0.0;
  }

  public DriveSpeed(double vx, double vy, double zRotation) {
    this.vx = vx;
    this.vy = vy;
    this.zRotation = zRotation;
  }
}
