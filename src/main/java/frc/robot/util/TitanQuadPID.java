package frc.robot.util;

import com.studica.frc.TitanQuad;
import edu.wpi.first.wpilibj.controller.PIDController;


public class TitanQuadPID extends TitanQuad {
  private final PIDController controller;

  private final double kP = 0.04;
  private final double kD = 0.004;

  public TitanQuadPID(int deviceID, int frequency, int motor) {
    super(deviceID, frequency, motor);
    controller = new PIDController(kP, 0.0, kD);
  }

  public TitanQuadPID(int deviceID, int motor) {
    super(deviceID, motor);
    controller = new PIDController(kP, 0.0, kD);
  }

  @Override
  public void set(double speed) {
    final var rpm = getRPM();
    final var target = speed * 80;
    final var fix = controller.calculate(rpm, target);
    super.set(speed + fix);
  }
}
