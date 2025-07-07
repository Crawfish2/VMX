package frc.robot.subsystems;

import com.studica.frc.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemExBase;
import static frc.robot.Constants.ArmConstants.SERVO_CHANNEL;

public class Arm extends SubsystemExBase {
  private final Servo servo;
  private static final double GRAB = 80;
  private static final double RELEASE = 0;

  public Arm() {
    servo = new Servo(SERVO_CHANNEL);
    // サーボのセーフティを有効にすると、ものを掴んだあと、すぐに放してしまう
    // 掴み続けるために、サーフティを無効にする
    servo.setSafetyEnabled(false);

    registerToShuffleboard();
  }

  /** Shuffleboardへの登録をする */
  private void registerToShuffleboard() {
    SendableRegistry.addChild(this, servo);
    SendableRegistry.addLW(servo, getName());
  }

  /** ものを掴む(アームを閉じる) */
  public void grab() {
    servo.setAngle(GRAB);
  }

  /** 掴んだものを放す(アームを開く) */
  public void release() {
    servo.setAngle(RELEASE);
  }

  /**
   * アームのモーターを止める (ものを掴んでいても、放してしまうので注意)
   */
  public void stopMotor() {
    servo.stopMotor();
  }

  /**
   * ものを掴み続けるコマンド
   * 終了するとつかむのをやめる
   */
  public CommandBase GrabCommand() {
    return runEnd(this::grab, this::stopMotor);
  }

  /**
   * ものを放す(アームを開き続ける)コマンド
   * 終了するとアームを開くのをやめる
   */
  public CommandBase ReleaseCommand() {
    return runEnd(this::release, this::stopMotor);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Angle", servo::getAngle, servo::setAngle);
  }
}
