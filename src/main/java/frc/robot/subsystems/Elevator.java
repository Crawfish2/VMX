package frc.robot.subsystems;

import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemExBase;
import static frc.robot.Constants.TitanConstants.TITAN_ID;
import static frc.robot.Constants.TitanConstants.ElevatorConstants.ELEVATOR;
import static frc.robot.Constants.TitanConstants.ElevatorConstants.WHEEL_DIST_PER_TICK;


/** アームのエレベーター */
// TODO: エレベーターのテストをする
public class Elevator extends SubsystemExBase {
  private final TitanQuad motor;
  private final TitanQuadEncoder encoder;
  private final SaferMotor drive;

  private static final double SPEED = 0.3;

  public Elevator() {
    motor = new TitanQuad(TITAN_ID, ELEVATOR);
    encoder = new TitanQuadEncoder(motor, ELEVATOR, WHEEL_DIST_PER_TICK);
    drive = new SaferMotor(motor);

    SendableRegistry.addChild(this, drive);

    Shuffleboard.getTab("Titan").add(this);
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
  public double getPosition() {
    return encoder.getEncoderDistance();
  }

  /** エレベーターのエンコーダー値をリセットする */
  public void resetPosition() {
    encoder.reset();
  }

  /** エレベーターのモーターを停止する */
  public void stopMotor() {
    drive.stopMotor();
  }

  /**
   * エレベーターを指定した位置まで上げるコマンド
   *
   * @param position 最低でも上がってほしい位置
   */
  public Command RaiseElevatorCommand(double position) {
    return runDeadline(() -> drive.drive(SPEED),
        () -> getPosition() > position);
  }

  /**
   * エレベーターを指定した位置まで下げるコマンド
   *
   * @param position 最高でも下がってほしい位置
   */
  public Command LowerElevatorCommand(double position) {
    return runDeadline(() -> drive.drive(SPEED),
        () -> getPosition() < position);
  }

  /**
   * エレベーターを指定した位置まで動かすコマンド
   * エンコーダーが指定した位置より下にいれば上昇、上にいれば下降する
   *
   * @param position 移動してほしい位置
   */
  public Command ElevatorToPositionCommand(double position) {
    return new ConditionalCommand(RaiseElevatorCommand(position), LowerElevatorCommand(position),
        (() -> getPosition() > position));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Encoder", encoder::getEncoderDistance, null);
    builder.addDoubleProperty("Motor Speed", motor::get, motor::set);
  }
}


class SaferMotor extends MotorSafety implements Sendable, AutoCloseable {
  private final SpeedController motor;

  public SaferMotor(SpeedController motor) {
    this.motor = motor;
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
    builder.addDoubleProperty("Motor Speed", motor::get, motor::set);
  }
}
