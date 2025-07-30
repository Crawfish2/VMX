package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

// TODO: テスト、調整する
public class PositionDriver {
  private static enum Phase {
    START, ALIGN_HEADING, TRANSLATE_POSITION, FINAL_ADJUST, DONE
  }

  /** 位置の許容誤差（mm） */
  private static final double positionTolerance = 5;
  /** 角度の許容誤差（度） */
  private static final double angleTolerance = 2.0;

  // 最大速度
  private static final double maxSpeed = 0.5;
  /** 加速時間（秒） */
  private static final double accelerationTime = 3.0;

  /** 最終的な目標位置 */
  private Pose2d targetPose;

  /** 現在のフェーズ */
  private Phase phase;
  /** 現在のフェーズでの目標位置 */
  private Pose2d phaseTargetPose;
  /** フェーズ開始後からの経過時間用タイマー */
  private final Timer timer;

  /**
   * PositionDriverのコンストラクタ
   */
  public PositionDriver() {
    timer = new Timer();
    targetPose = new Pose2d();
    phase = Phase.DONE;
  }

  /**
   * タイマーをリセットする
   */
  private void resetPhaseTimer() {
    timer.reset();
  }

  /**
   * 目標位置を設定して開始する
   *
   * @param target 目標位置
   */
  public void setTarget(Pose2d target) {
    targetPose = target;
    phase = Phase.START;
  }

  /**
   * 移動速度を計算する
   *
   * @param currentPose オドメトリから取得した現在の位置
   * @return 目標に向かうための速度指令値
   */
  public DriveSpeed getVelocity(Pose2d currentPose) {
    switch (phase) {
      case START:
        // 次のフェーズに向けて初期化して移行する
        timer.start();

        phase = Phase.ALIGN_HEADING;
        // 現在の座標を維持しつつ回転する
        phaseTargetPose = new Pose2d(currentPose.getTranslation(), targetPose.getRotation());

        resetPhaseTimer();
        return getVelocity(currentPose);
      case ALIGN_HEADING:
        if (isAngleReached(currentPose)) {

          phase = Phase.TRANSLATE_POSITION;
          // 姿勢を維持しつつ平行移動する
          phaseTargetPose = new Pose2d(targetPose.getTranslation(), currentPose.getRotation());

          resetPhaseTimer();
          return getVelocity(currentPose);
        }
        return computeDriveVelocity(currentPose);
      case TRANSLATE_POSITION:
        if (isPositionReached(currentPose)) {
          phase = Phase.FINAL_ADJUST;
          // 最終的な目標位置、姿勢を目指す
          phaseTargetPose = targetPose;

          resetPhaseTimer();
          return getVelocity(currentPose);
        }
        return computeDriveVelocity(currentPose);
      case FINAL_ADJUST:
        if (isAngleReached(currentPose) && isPositionReached(currentPose)) {
          phase = Phase.DONE;
          // 参照はされないが、流れを統一するために、一応設定しておく
          phaseTargetPose = targetPose;

          resetPhaseTimer();
          return getVelocity(currentPose);
        }
        return computeDriveVelocity(currentPose);
      case DONE:
        timer.stop();
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
    Translation2d diff = phaseTargetPose.getTranslation().minus(currentPose.getTranslation());
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
    double targetAngleDeg = phaseTargetPose.getRotation().getDegrees();

    // 角度差を計算（度単位）
    double angleDiff = angleDiffDegrees(currentAngleDeg, targetAngleDeg);

    // 角速度計算
    double angularVelocity = applyAngularProfile(angleDiff);
    angularVelocity *= getAccelerationFactor();

    return angularVelocity;
  }


  /**
   * 直線、回転移動の速度を計算する
   *
   * @param currentPose 現在の位置
   * @return 移動速度 (RobotDrive)
   */
  private DriveSpeed computeDriveVelocity(Pose2d currentPose) {
    double[] linear = computeLinearVelocity(currentPose);
    double rotate = computeAngularVelocity(currentPose);
    return new DriveSpeed(linear[0], linear[1], rotate);
  }

  /**
   * 速度に非線形プロファイルを適用する
   *
   * @param distance 目標までの距離
   * @return 適用後の速度
   */
  private double applyVelocityProfile(double distance) {
    final double[] inRange = {0, 2, 2, 30, 50};
    final double[] outRange = {0, 0, 0.15, 0.2, 0.5};
    return TransferFunction.transferFunction(distance, inRange, outRange);
  }

  /**
   * 角速度に非線形プロファイルを適用する
   *
   * @param angleDiff 角度差（度）
   * @return 適用後の角速度（RobotDrive）
   */
  private double applyAngularProfile(double angleDiff) {
    final double[] inRange = {0, 0.5, 0.5, 20};
    final double[] outRange = {0, 0, 0.1, 0.3};

    final double vel = TransferFunction.transferFunction(Math.abs(angleDiff), inRange, outRange);
    return Math.copySign(vel, angleDiff);
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
   * 角度の差分(to - from)を計算する
   *
   * @param from もとの角度（度）
   * @param to 次の角度?（度）
   * @return -180 ～ +180 の範囲に正規化された角度の差分（度）
   */
  private static double angleDiffDegrees(double from, double to) {
    double diff = to - from;
    while (diff > 180)
      diff -= 360;
    while (diff < -180)
      diff += 360;
    return diff;
  }


  private boolean isPositionReached(Pose2d current) {
    var error = phaseTargetPose.getTranslation().minus(current.getTranslation());
    return Math.abs(error.getX()) < positionTolerance
        && Math.abs(error.getY()) < positionTolerance;
  }

  private boolean isAngleReached(Pose2d current) {
    double error =
        angleDiffDegrees(current.getRotation().getDegrees(),
            phaseTargetPose.getRotation().getDegrees());
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
