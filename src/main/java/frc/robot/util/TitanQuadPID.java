package frc.robot.util;

import com.studica.frc.TitanQuad;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;


public class TitanQuadPID extends TitanQuad implements Sendable {
  private final PIDController controller;

  private final double kP = 0.3;
  private final double kD = 0.002;

  public TitanQuadPID(int deviceID, int frequency, int motor) {
    super(deviceID, frequency, motor);
    controller = new PIDController(kP, 0.0, kD);
    SendableRegistry.addChild(this, controller);
  }

  public TitanQuadPID(int deviceID, int motor) {
    super(deviceID, motor);
    controller = new PIDController(kP, 0.0, kD);
    SendableRegistry.addChild(this, controller);
  }

  @Override
  public void set(double speed) {
    final var maxRPM = 120;
    final var rpm = (double) -getRPM() / maxRPM;
    final var target = speed;
    final var fix = controller.calculate(rpm, target);
    final var setSpeed = speed + fix;
    super.set(setSpeed);
  }

  @Override
  public void stopMotor() {
    super.stopMotor();
    controller.calculate(0, 0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {}
}
