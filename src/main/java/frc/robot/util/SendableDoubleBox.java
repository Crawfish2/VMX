package frc.robot.util;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class SendableDoubleBox extends Box<Double> implements Sendable {
  public SendableDoubleBox(double value) {
    super(value);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("value", this::get, this::set);
  }
}
