package frc.robot.util;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class SaferMotor extends MotorSafety implements Sendable, AutoCloseable {
  private final SpeedController motor;

  public SaferMotor(SpeedController motor) {
    this.motor = motor;

    // 一定時間経過後に、自動で停止する
    setSafetyEnabled(true);

    registerToShuffleboard();
  }

  /** Shuffleboardへの登録をする */
  private void registerToShuffleboard() {
    SendableRegistry.addChild(this, this.motor);
    SendableRegistry.addLW(this, "SaferMotor");
  }

  public void drive(double speed) {
    motor.set(speed);
    feed();
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  @Override
  public String getDescription() {
    return "SaferMotor";
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
    feed();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SaferMotor");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Motor Speed", motor::get, this::drive);
  }
}
