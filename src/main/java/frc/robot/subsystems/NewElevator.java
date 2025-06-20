package frc.robot.subsystems;

import com.studica.frc.ServoContinuous;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemExBase;

/** アームのエレベーター */
// TODO: エレベーターのテストをする
public class NewElevator extends SubsystemExBase {
  private final ServoContinuous servo;
  private final SaferMotor drive;

  private static final double SPEED = 0.3;

  private static final int ELEVATOR_CHANNEL = 3;

  public NewElevator() {
    servo = new ServoContinuous(ELEVATOR_CHANNEL);
    drive = new SaferMotor(servo);

    registerToShuffleboard();
  }

  /** Shuffleboardへの登録をする */
  private void registerToShuffleboard() {
    SendableRegistry.addChild(this, drive);

    Shuffleboard.getTab("Elevator").add(this);
  }

  /** エレベーターを上げる */
  public void raise() {
    drive.drive(SPEED);
  }

  /** エレベーターを下げる */
  public void lower() {
    drive.drive(-SPEED);
  }

  /** エレベーターのエンコーダー値を取得する */
  @Deprecated
  public double getPosition() {
    return 0.0;
    // return encoder.getEncoderDistance();
  }

  /** エレベーターのエンコーダー値をリセットする */
  @Deprecated
  public void resetPosition() {
    // encoder.reset();
  }

  /** エレベーターのモーターを停止する */
  public void stopMotor() {
    drive.stopMotor();
  }

  /** エレベーターのエンコーダーの距離をリセットするコマンド */
  @Deprecated
  public CommandBase ResetElevatorEncoderCommand() {
    return runOnce(() -> {
    });
    // return runOnce(encoder::reset);
  }

  /**
   * エレベーターを指定した位置まで上げるコマンド
   *
   * @param position 最低でも上がってほしい位置
   */
  public CommandBase RaiseElevatorCommand(double position) {
    return runDeadline(this::raise,
        () -> getPosition() > position);
  }

  /**
   * エレベーターを指定した位置まで下げるコマンド
   *
   * @param position 最高でも下がってほしい位置
   */
  public CommandBase LowerElevatorCommand(double position) {
    return runDeadline(this::lower,
        () -> getPosition() < position);
  }

  /**
   * エレベーターを指定した位置まで動かすコマンド
   * エンコーダーが指定した位置より下にいれば上昇、上にいれば下降する
   *
   * @param position 移動してほしい位置
   */
  public CommandBase ElevatorToPositionCommand(double position) {
    return new ConditionalCommand(RaiseElevatorCommand(position), LowerElevatorCommand(position),
        (() -> getPosition() < position));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    // builder.addDoubleProperty("Encoder", encoder::getEncoderDistance, null);
    builder.addDoubleProperty("Motor Speed", servo::get, drive::drive);
  }
}
