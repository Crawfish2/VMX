package frc.robot.subsystems;

import com.studica.frc.TitanQuadEncoder;
import edu.wpi.first.wpilibj.drive.KilloughDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemExBase;
import frc.robot.util.Odometry;
import frc.robot.util.PositionDriver;
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
import java.util.function.Supplier;

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

  /** コマンド呼び出しのみpublicとして使うこと */
  public final Odometry odometry;

  /** コマンド呼び出しのみpublicとして使うこと */
  public final PositionDriver posDriver;

  private double deadband = 0.02;
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

    posDriver = new PositionDriver();

    registerToShuffleboard();
  }

  /** Shuffleboardへの登録をする */
  private void registerToShuffleboard() {
    SendableRegistry.addChild(this, drive);
    SendableRegistry.addChild(this, odometry);

    final var tab = Shuffleboard.getTab("Titan");

    tab.add(this);
    tab.add(drive);
    tab.add(odometry);

    tab.add(withName("Odometry Reset Pose", odometry.ResetPoseCommand(new Pose2d())));

    final var targetX = tab.add("Pos Driver Target X", 0.0).getEntry();
    final var targetY = tab.add("Pos Driver Target Y", 0.0).getEntry();
    final var targetAngle = tab.add("Pos Driver Target Angle", 0.0).getEntry();

    tab.add(withName("Test Pos Driver", defer(() -> moveToPoseCommand(
        new Pose2d(targetX.getDouble(0.0), targetY.getDouble(0.0),
            Rotation2d.fromDegrees(targetAngle.getDouble(0.0))),
        odometry::getPose))));
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
   *
   * @deprecated 代わりに.odometry.ResetPoseCommand()を使用すること
   */
  @Deprecated
  public Command ResetEncodersDistanceCommand() {
    return runOnce(this::resetEncodersDistance);
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
   * 指定した速度で、回転し続けるコマンド
   *
   * @param angle [-1..1] 回転する速度
   */
  public Command RotateCommand(double angle) {
    return run(() -> driveCartesian(0, 0, angle));
  }

  public CommandBase moveToPoseCommand(Pose2d target, Supplier<Pose2d> currentPose) {
    return functional(() -> posDriver.setTarget(target),
        () -> {
          final var velocity = posDriver.getVelocity(currentPose.get());
          drive.driveCartesian(velocity.vy, velocity.vx,
              velocity.zRotation);
        }, null,
        posDriver::isCompleted);
  }


  @Override
  public void periodic() {
    odometry.update(
        encoderLeft.getEncoderDistance(),
        encoderRight.getEncoderDistance(),
        encoderBack.getEncoderDistance());
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
    builder.addDoubleProperty("Motor Left RPM", motorLeft::getRPM, null);
    builder.addDoubleProperty("Motor Right RPM", motorRight::getRPM, null);
    builder.addDoubleProperty("Motor Back RPM", motorBack::getRPM, null);
    builder.addDoubleProperty("Deadband", () -> deadband, this::setDeadband);
    builder.addDoubleProperty("MaxOutput", () -> maxOutput, this::setMaxOutput);
  }
}
