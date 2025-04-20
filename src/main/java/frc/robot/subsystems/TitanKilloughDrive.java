package frc.robot.subsystems;

import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import edu.wpi.first.wpilibj.drive.KilloughDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemExBase;
import static frc.robot.Constants.TITAN_ID;
import static frc.robot.Constants.WHEEL_DIST_PER_TICK;
import static frc.robot.Constants.WHEEL_FRONT;
import static frc.robot.Constants.WHEEL_FRONT_Angle;
import static frc.robot.Constants.WHEEL_LEFT;
import static frc.robot.Constants.WHEEL_LEFT_Angle;
import static frc.robot.Constants.WHEEL_RIGHT;
import static frc.robot.Constants.WHEEL_RIGHT_Angle;

/**
 * ロボットのホイールを制御する
 */
public class TitanKilloughDrive extends SubsystemExBase {
  private final TitanQuad motorFront;
  private final TitanQuad motorLeft;
  private final TitanQuad motorRight;

  private final TitanQuadEncoder encoderFront;
  private final TitanQuadEncoder encoderLeft;
  private final TitanQuadEncoder encoderRight;

  private final KilloughDrive drive;

  private double deadband = 0.05;
  private double maxOutput = 1.0;

  public TitanKilloughDrive() {
    motorLeft = new TitanQuad(TITAN_ID, WHEEL_LEFT);
    motorRight = new TitanQuad(TITAN_ID, WHEEL_RIGHT);
    motorFront = new TitanQuad(TITAN_ID, WHEEL_FRONT);

    encoderLeft = new TitanQuadEncoder(motorLeft, WHEEL_LEFT, WHEEL_DIST_PER_TICK);
    encoderRight = new TitanQuadEncoder(motorRight, WHEEL_RIGHT, WHEEL_DIST_PER_TICK);
    encoderFront = new TitanQuadEncoder(motorFront, WHEEL_FRONT, WHEEL_DIST_PER_TICK);

    drive = new KilloughDrive(
        motorLeft, motorRight, motorFront,
        WHEEL_LEFT_Angle, WHEEL_RIGHT_Angle, WHEEL_FRONT_Angle);
    drive.setDeadband(deadband);
    drive.setMaxOutput(maxOutput);

    Shuffleboard.getTab("Titan").add(this);
  }

  public void setDeadband(double deadband) {
    drive.setDeadband(deadband);
    this.deadband = deadband;
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
    this.maxOutput = maxOutput;
  }

  /**
   * X方向、Y方向を指定して進む
   *
   * @param ySpeed [-1.0..1.0] 左右の速度、正の方向は右
   * @param xSpeed [-1.0..1.0] 前後の速度、正の方向は前
   * @param zRotation [-1.0..1.0] 回転速度、正の方向は時計回り
   */
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
    drive.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  /**
   * @param ySpeed [-1.0..1.0] 左右の速度、正の方向は右
   * @param xSpeed [-1.0..1.0] 前後の速度、正の方向は前
   * @param zRotation [-1.0..1.0] 回転速度、正の方向は時計回り
   * @param gyroAngle [deg] 現在のロボットの角度、X軸の正方向が0度
   */
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
    drive.driveCartesian(ySpeed, xSpeed, zRotation, gyroAngle);
  }

  /**
   * 速度、角度を指定して進む
   *
   * @param magnitude [-1.0..1.0] 進行方向の速度、角度に対して正の方向は前
   * @param angle [-180..180] 進行方向の角度
   * @param zRotation [-1.0..1.0] 回転速度、正の方向は時計回り
   */
  public void drivePolar(double magnitude, double angle, double zRotation) {
    drive.drivePolar(magnitude, angle, zRotation);
  }

  /**
   * 直進するコマンド
   * 0で前進、90で右、-90で左に移動する。
   * 角度は度数法で指定する。
   */
  public Command DriveCommand(double angle) {
    final double speed = 0.3;
    return run(() -> drivePolar(speed, angle, 0));
  }

  /**
   * モーターを停止する
   */
  public void stopMotor() {
    drive.stopMotor();
  }

  /**
   * エンコーダーの距離をリセットする
   */
  public void resetEncodersDistance() {
    encoderLeft.reset();
    encoderRight.reset();
    encoderFront.reset();
  }

  /**
   * 進んだ距離を角度指定で取得する
   *
   * @apiNote 回転を含む移動をした場合、進んだ距離は正確に取得できない
   * @param angle [-180..180] 取得したい角度
   * @return 指定した角度の方向に進んだ距離
   */
  public double getDistancePolar(double angle) {
    // 各モーターのエンコーダー距離を取得
    double d_l = encoderLeft.getEncoderDistance();
    double d_r = encoderRight.getEncoderDistance();
    double d_f = encoderFront.getEncoderDistance();

    // 各ホイールの寄与を計算
    double contribution_left = d_l * Math.sin(Math.toRadians(angle + WHEEL_LEFT_Angle));
    double contribution_right = d_r * Math.sin(Math.toRadians(angle + WHEEL_RIGHT_Angle));
    double contribution_front = d_f * Math.sin(Math.toRadians(angle + WHEEL_FRONT_Angle));

    // 3つの寄与の平均を取ることで、指定方向の距離を得る
    // FIXME: 距離が実際よりもかなり小さい値を返すので、修正する
    return (contribution_left + contribution_right + contribution_front) / 3.0;
  }

  /**
   * 回転する距離を取得する
   *
   * @param angle [-180..180] 回転する角度
   * @return 指定した角度の回転距離
   */
  public double getRotateDistance(double angle) {
    double distance = Math.toRadians(angle) * 245;
    return distance;
  }

  /**
   * 回転するコマンド
   *
   * @param angle [-180..180] 回転する角度
   */
  public Command RotateDistanceCommand(double angle) {
    double speed = Math.copySign(0.3, angle); // 左(負)向きなら、左回転する

    double distance = getRotateDistance(Math.abs(angle));
    return functional(this::resetEncodersDistance,
        () -> drivePolar(0, 0, speed), null,
        // TODO: 3つのエンコーダーの距離の平均をとるようにする
        () -> Math.abs(encoderLeft.getEncoderDistance()) > distance);
  }

  /**
   * ShuffleBoardなどに表示するときの設定をする
   * (勝手に呼ばれるので気にしなくてよい)
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Encoder Left", encoderLeft::getEncoderDistance, null);
    builder.addDoubleProperty("Encoder Right", encoderRight::getEncoderDistance, null);
    builder.addDoubleProperty("Encoder Front", encoderFront::getEncoderDistance, null);
    builder.addDoubleProperty("Deadband", () -> deadband, this::setDeadband);
    builder.addDoubleProperty("MaxOutput", () -> maxOutput, this::setMaxOutput);
  }
}
