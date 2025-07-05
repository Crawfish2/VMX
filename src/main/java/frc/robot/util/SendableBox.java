package frc.robot.util;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * Sendableを実装した値のラッパー
 * TはSendableを実装している必要がある
 */
public class SendableBox<T extends Sendable> extends Box<T> implements Sendable {

  public SendableBox(T value) {
    super(value);
    SendableRegistry.addChild(this, value);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // 内部のTオブジェクトのSendable実装を委譲
    if (get() != null) {
      get().initSendable(builder);
    }
  }
}
