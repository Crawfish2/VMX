package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

// TODO: テスト、調整する
public class PositionDriver {
  private static enum Phase {
    ALIGN_HEADING, TRANSLATE_POSITION, FINAL_ADJUST, DONE
  }

  /** 位置の許容誤差（mm） */
  private static final double positionTolerance = 5;
  /** 角度の許容誤差（度） */
  private static final double angleTolerance = 2.0;

  // 最大速度
  private static final double maxSpeed = 0.5;
  /** 加速時間（秒） */
  private static final double accelerationTime = 3.0;

  private Pose2d targetPose;
  private final Timer timer;

  private Phase phase;

  /**
   * PositionDriverのコンストラクタ
   */
  public PositionDriver() {
    timer = new Timer();
    targetPose = new Pose2d();

    timer.start();
    restart();
  }

  /**
   * 制御を初期状態に戻す
   */
  private void restart() {
    timer.reset();
    phase = Phase.ALIGN_HEADING;
  }

  /**
   * 目標位置を設定する
   *
   * @param target 目標位置
   */
  public void setTarget(Pose2d target) {
    targetPose = target;
    restart();
  }


  /**
   * 移動速度を計算する
   *
   * @param currentPose オドメトリから取得した現在の位置
   * @return 目標に向かうための速度指令値
   */
  public DriveSpeed getVelocity(Pose2d currentPose) {
    switch (phase) {
      case ALIGN_HEADING:
        if (isAngleReached(currentPose)) {
          phase = Phase.FINAL_ADJUST;
          timer.reset();
          return getVelocity(currentPose);
        }
        return new DriveSpeed(0, 0, computeAngularVelocity(currentPose));

      case TRANSLATE_POSITION:
        if (isPositionReached(currentPose)) {
          phase = Phase.FINAL_ADJUST;
          timer.reset();
          return getVelocity(currentPose);
        }
        double[] linear = computeLinearVelocity(currentPose);
        return new DriveSpeed(linear[0], linear[1], 0);

      case FINAL_ADJUST:
        boolean angleDone = isAngleReached(currentPose);
        boolean posDone = isPositionReached(currentPose);
        if (angleDone && posDone) {
          phase = Phase.DONE;
          return new DriveSpeed();
        }
        double[] linAdj = computeLinearVelocity(currentPose);
        double angAdj = computeAngularVelocity(currentPose);
        return new DriveSpeed(linAdj[0], linAdj[1], angAdj);

      case DONE:
      default:
        return new DriveSpeed();
    }
  }

  /**
   * 直線移動速度を計算する
   *
   * @param currentPose 現在の位置
   * @return [vx, vy]の速度ベクトル
   */
  private double[] computeLinearVelocity(Pose2d currentPose) {
    double currentAngle = currentPose.getRotation().getRadians();

    // 極座標に変換
    Translation2d diff = targetPose.getTranslation().minus(currentPose.getTranslation());
    double distance = diff.getNorm();
    double angle = Math.atan2(diff.getY(), diff.getX());

    // 速度調整
    double velocity = applyVelocityProfile(distance);
    velocity *= getAccelerationFactor();
    velocity = Math.min(velocity, maxSpeed);

    // 現在の姿勢を考慮して速度を変換
    angle -= currentAngle;
    double vx = velocity * Math.cos(angle);
    double vy = velocity * Math.sin(angle);

    return new double[] {vx, vy};
  }

  /**
   * 回転速度を計算する
   *
   * @param currentPose 現在の位置
   * @return 回転速度（RobotDrive）
   */
  private double computeAngularVelocity(Pose2d currentPose) {
    double currentAngleDeg = currentPose.getRotation().getDegrees();
    double targetAngleDeg = targetPose.getRotation().getDegrees();

    // 角度差を計算（度単位）
    double angleDiff = targetAngleDeg - currentAngleDeg;
    // -180 ～ +180 の範囲に正規化
    while (angleDiff > 180)
      angleDiff -= 360;
    while (angleDiff < -180)
      angleDiff += 360;

    // 角速度計算
    double angularVelocity = applyAngularProfile(angleDiff);
    angularVelocity *= getAccelerationFactor();

    return angularVelocity;
  }

  /**
   * 速度に非線形プロファイルを適用する
   *
   * @param distance 目標までの距離
   * @return 適用後の速度
   */
  // TODO: 実装を置き換える
  private double applyVelocityProfile(double distance) {
    // 簡易的な実装
    // 実際のロボットの特性に合わせて調整が必要
    if (distance < 10)
      return 0.05;
    if (distance < 75)
      return 0.15;
    if (distance < 300)
      return 0.3;
    return 0.5; // 最大速度
  }

  /**
   * 角速度に非線形プロファイルを適用する
   *
   * @param angleDiff 角度差（度）
   * @return 適用後の角速度（RobotDrive）
   */
  // TODO: 実装を置き換える
  private double applyAngularProfile(double angleDiff) {
    // 簡易的な実装
    if (Math.abs(angleDiff) < 0.5)
      return 0;
    if (Math.abs(angleDiff) < 5)
      return Math.copySign(0.05, angleDiff);
    if (Math.abs(angleDiff) < 30)
      return Math.copySign(0.10, angleDiff);
    return Math.copySign(0.15, angleDiff); // 最大角速度
  }

  /**
   * 加速時間に基づく係数を計算する
   *
   * @return 加速係数（0〜1）
   */
  private double getAccelerationFactor() {
    return Math.min(timer.get() / accelerationTime, 1.0);
  }

  private double angleDiffDegrees(double from, double to) {
    double diff = to - from;
    while (diff > 180)
      diff -= 360;
    while (diff < -180)
      diff += 360;
    return diff;
  }


  private boolean isPositionReached(Pose2d current) {
    var error = targetPose.getTranslation().minus(current.getTranslation());
    return Math.abs(error.getX()) < positionTolerance
        && Math.abs(error.getY()) < positionTolerance;
  }

  private boolean isAngleReached(Pose2d current) {
    double error =
        angleDiffDegrees(current.getRotation().getDegrees(), targetPose.getRotation().getDegrees());
    return Math.abs(error) < angleTolerance;
  }

  /**
   * 目標に到達したかどうか
   *
   * @param currentPose 現在の位置
   * @return 到達していればtrue
   */
  public boolean isCompleted() {
    return Phase.DONE.equals(phase);
  }
}
