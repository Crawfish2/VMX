package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Units;

// TODO: テスト、調整する
public class PositionDriver {
  /** 位置の許容誤差（mm） */
  private final double positionTolerance = 5;
  /** 角度の許容誤差（度） */
  private final double directionTolerance = 2.0;
  /** 加速時間（秒） */
  private final double accelerationTime = 3.0;

  private Pose2d targetPose;
  private final Timer timer;

  // flags
  private boolean positionReached;
  private boolean directionReached;
  private boolean completed;

  /**
   * PositionDriverのコンストラクタ
   */
  public PositionDriver() {
    timer = new Timer();
    targetPose = new Pose2d();
    restart();
  }

  /**
   * 制御を初期状態に戻す
   */
  public void restart() {
    timer.reset();
    timer.start();
    positionReached = false;
    directionReached = false;
    completed = false;
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
   * 目標位置を設定する（mm単位での指定）
   *
   * @param x X座標（mm単位）
   * @param y Y座標（mm単位）
   * @param degrees 向き（度単位）
   */
  public void setTargetPosition(double x, double y, double degrees) {
    targetPose = new Pose2d(
        x,
        y,
        Rotation2d.fromDegrees(degrees));
    restart();
  }

  /**
   * 移動速度を計算する
   *
   * @param currentPose オドメトリから取得した現在の位置
   * @return 目標に向かうための速度指令値
   */
  public ChassisSpeeds getVelocity(Pose2d currentPose) {
    // 既に完了していたら停止
    if (isReached()) {
      return new ChassisSpeeds(0, 0, 0);
    }

    // まず向きを合わせる
    if (!isDirectionReached(currentPose)) {
      double angularVelocity = calculateAngularVelocity(currentPose);
      return new ChassisSpeeds(0, 0, angularVelocity);
    }

    // 向きが合っていれば位置へ移動
    double[] linearVelocities = calculateLinearVelocity(currentPose);
    return new ChassisSpeeds(linearVelocities[0], linearVelocities[1], 0);
  }

  /**
   * 直線移動速度を計算する
   *
   * @param currentPose 現在の位置
   * @return [vx, vy]の速度ベクトル
   */
  private double[] calculateLinearVelocity(Pose2d currentPose) {
    double currentX = currentPose.getTranslation().getX();
    double currentY = currentPose.getTranslation().getY();
    double currentAngle = currentPose.getRotation().getRadians();

    // 目標との差分を計算
    double xDiff = targetPose.getTranslation().getX() - currentX;
    double yDiff = targetPose.getTranslation().getY() - currentY;

    // 極座標に変換
    double distance = Math.sqrt(xDiff * xDiff + yDiff * yDiff);
    double angle = Math.atan2(yDiff, xDiff);

    // 速度調整
    double velocity = applyVelocityProfile(distance);
    velocity *= getAccelerationFactor();

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
   * @return 角速度（ラジアン/秒）
   */
  private double calculateAngularVelocity(Pose2d currentPose) {
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

    // ラジアンに変換
    return Units.degreesToRadians(angularVelocity);
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
    if (distance < 0.01)
      return 0.05;
    if (distance < 0.05)
      return 0.15;
    if (distance < 0.1)
      return 0.3;
    return 0.5; // 最大速度
  }

  /**
   * 角速度に非線形プロファイルを適用する
   *
   * @param angleDiff 角度差（度）
   * @return 適用後の角速度（度/秒）
   */
  // TODO: 実装を置き換える
  private double applyAngularProfile(double angleDiff) {
    // 簡易的な実装
    if (Math.abs(angleDiff) < 0.5)
      return 0;
    if (Math.abs(angleDiff) < 5)
      return Math.signum(angleDiff) * 10;
    if (Math.abs(angleDiff) < 20)
      return Math.signum(angleDiff) * 30;
    return Math.signum(angleDiff) * 50; // 最大角速度
  }

  /**
   * 加速時間に基づく係数を計算する
   *
   * @return 加速係数（0〜1）
   */
  private double getAccelerationFactor() {
    return Math.min(timer.get() / accelerationTime, 1.0);
  }

  /**
   * XY位置が目標に到達したかどうか
   *
   * @param currentPose 現在の位置
   * @return 到達していればtrue
   */
  public boolean isPositionReached(Pose2d currentPose) {
    if (positionReached) {
      return true;
    }

    final var targetTranslation = targetPose.getTranslation();
    final var currentTranslation = currentPose.getTranslation();

    double positionErrorX = Math.abs(targetTranslation.getX() - currentTranslation.getX());
    double positionErrorY = Math.abs(targetTranslation.getY() - currentTranslation.getY());

    boolean reached = (positionErrorX < positionTolerance &&
        positionErrorY < positionTolerance);

    if (reached) {
      positionReached = true;
      timer.reset();
    }

    return positionReached;
  }

  /**
   * 向きが目標に到達したかどうか
   *
   * @param currentPose 現在の位置
   * @return 到達していればtrue
   */
  public boolean isDirectionReached(Pose2d currentPose) {
    if (directionReached) {
      return true;
    }

    double currentAngleDeg = currentPose.getRotation().getDegrees();
    double targetAngleDeg = targetPose.getRotation().getDegrees();
    double directionError = Math.abs(targetAngleDeg - currentAngleDeg);

    // 180度を超える角度差は短い方向で計算
    if (directionError > 180) {
      directionError = 360 - directionError;
    }

    boolean reached = directionError < directionTolerance;

    if (reached) {
      directionReached = true;
      timer.reset();
    }

    return directionReached;
  }

  /**
   * 位置と向きの両方が目標に到達したかどうか
   *
   * @param currentPose 現在の位置
   * @return 到達していればtrue
   */
  public boolean isReached(Pose2d currentPose) {
    if (completed) {
      return true;
    }

    completed = isPositionReached(currentPose) && isDirectionReached(currentPose);
    return completed;
  }

  /**
   * 現在の到達状態を返す
   *
   * @return 到達していればtrue
   */
  public boolean isReached() {
    return completed;
  }
}
