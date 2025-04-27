package frc.robot.subsystems;

import com.studica.frc.TitanQuadEncoder;
import edu.wpi.first.wpilibj.drive.KilloughDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemExBase;
import frc.robot.util.Odometry;
import frc.robot.util.TitanQuadPID;
import static frc.robot.Constants.TitanConstants.TITAN_ID;
import static frc.robot.Constants.TitanConstants.DriveConstants.WHEEL_DIST_PER_TICK;
import static frc.robot.Constants.TitanConstants.DriveConstants.WHEEL_BACK;
import static frc.robot.Constants.TitanConstants.DriveConstants.WHEEL_BACK_ANGLE;
import static frc.robot.Constants.TitanConstants.DriveConstants.WHEEL_LEFT;
import static frc.robot.Constants.TitanConstants.DriveConstants.WHEEL_LEFT_ANGLE;
import static frc.robot.Constants.TitanConstants.DriveConstants.WHEEL_RIGHT;
import static frc.robot.Constants.TitanConstants.DriveConstants.WHEEL_RIGHT_ANGLE;
import static frc.robot.Constants.TitanConstants.DriveConstants.wheelDistanceFromCenter;

/**
 * ロボットのホイールを制御する
 */
public class TitanKilloughDrive extends SubsystemExBase {
  private final TitanQuadPID motorBack;
  private final TitanQuadPID motorLeft;
  private final TitanQuadPID motorRight;

  private final TitanQuadEncoder encoderBack;
  private final TitanQuadEncoder encoderLeft;
  private final TitanQuadEncoder encoderRight;

  private final KilloughDrive drive;

  private final Odometry odometry;

  private double deadband = 0.05;
  private double maxOutput = 1.0;

  public TitanKilloughDrive() {
    motorLeft = new TitanQuadPID(TITAN_ID, WHEEL_LEFT);
    motorRight = new TitanQuadPID(TITAN_ID, WHEEL_RIGHT);
    motorBack = new TitanQuadPID(TITAN_ID, WHEEL_BACK);

    encoderLeft = new TitanQuadEncoder(motorLeft, WHEEL_LEFT, WHEEL_DIST_PER_TICK);
    encoderRight = new TitanQuadEncoder(motorRight, WHEEL_RIGHT, WHEEL_DIST_PER_TICK);
    encoderBack = new TitanQuadEncoder(motorBack, WHEEL_BACK, WHEEL_DIST_PER_TICK);

    resetEncodersDistance();

    drive = new KilloughDrive(
        motorLeft, motorRight, motorBack,
        WHEEL_LEFT_ANGLE, WHEEL_RIGHT_ANGLE, WHEEL_BACK_ANGLE);
    drive.setDeadband(deadband);
    drive.setMaxOutput(maxOutput);

    odometry = new Odometry();

    SendableRegistry.addChild(this, drive);
    SendableRegistry.addChild(this, odometry);

    final var tab = Shuffleboard.getTab("Titan");

    tab.add(this);
    tab.add(drive);
    tab.add(odometry);
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
   * 速度、向き、回転速度を指定して進む
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
   *
   * @param angle [-180..180] 進行方向の角度
   */
  public Command DriveCommand(double angle) {
    final double speed = 0.3;
    return DrivePolarCommand(speed, angle, 0);
  }

  /**
   * 速度、向き、回転速度を指定して進むコマンド
   *
   * @param speed [-1.0..1.0] 移動速度
   * @param angle [-180..180] 進行方向の角度
   * @param rotation [-1.0..1.0] 回転速度、正の方向は時計回り
   */
  public Command DrivePolarCommand(double speed, double angle, double rotation) {
    return run(() -> drivePolar(speed, angle, rotation));
  }

  /**
   * エンコーダーを値をもとに、指定した距離進むコマンド
   * コマンド初期化時に、エンコーダーの値をリセットする
   *
   * @param angle [-180..180] 進行方向の速度、角度に対して正の方向は前
   * @param distance [mm] 移動したい距離
   */
  public Command DriveDistanceCommand(double angle, double distance) {
    final double speed = 0.3;
    return functional(this::resetEncodersDistance, () -> drivePolar(speed, angle, 0), null,
        () -> getDistancePolar(angle) > distance);
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
    encoderBack.reset();
  }

  /**
   * エンコーダーの距離をリセットするコマンド
   */
  public Command ResetEncodersDistanceCommand() {
    return runOnce(this::resetEncodersDistance);
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
    double d_b = encoderBack.getEncoderDistance();

    // 各ホイールの寄与を計算
    double contribution_left = d_l * Math.sin(Math.toRadians(angle + WHEEL_LEFT_ANGLE));
    double contribution_right = d_r * Math.sin(Math.toRadians(angle + WHEEL_RIGHT_ANGLE));
    double contribution_back = d_b * Math.sin(Math.toRadians(angle + WHEEL_BACK_ANGLE));

    // 3つの寄与の平均を取ることで、指定方向の距離を得る
    // FIXME: 距離が実際よりもかなり小さい値を返すので、修正する
    return (contribution_left + contribution_right + contribution_back) / 3.0;
  }

  /**
   * 回転する距離を取得する
   *
   * @param angle [-180..180] 回転する角度
   * @return 指定した角度の回転距離
   */
  public double getRotateDistance(double angle) {
    double distance = Math.toRadians(angle) * wheelDistanceFromCenter;
    return distance;
  }

  /**
   * 指定した角度だけ回転するコマンド
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
   * 指定した速度で、回転し続けるコマンド
   *
   * @param angle [-1..1] 回転する速度
   */
  public Command RotateCommand(double angle) {
    return run(() -> driveCartesian(0, 0, angle));
  }

  @Override
  public void periodic() {
    odometry.update(
        encoderLeft.getEncoderDistance(),
        encoderLeft.getEncoderDistance(),
        encoderLeft.getEncoderDistance());
  }

  /**
   * ShuffleBoardなどに表示するときの設定をする
   * (勝手に呼ばれるので気にしなくてよい)
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Encoder Left", encoderLeft::getEncoderDistance, null);
    builder.addDoubleProperty("Encoder Right", encoderRight::getEncoderDistance, null);
    builder.addDoubleProperty("Encoder Back", encoderBack::getEncoderDistance, null);
    builder.addDoubleProperty("Deadband", () -> deadband, this::setDeadband);
    builder.addDoubleProperty("MaxOutput", () -> maxOutput, this::setMaxOutput);
  }
}
