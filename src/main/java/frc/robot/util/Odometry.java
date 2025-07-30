package frc.robot.util;

import static frc.robot.Constants.TitanConstants.DriveConstants.WHEEL_BACK_ANGLE;
import static frc.robot.Constants.TitanConstants.DriveConstants.WHEEL_LEFT_ANGLE;
import static frc.robot.Constants.TitanConstants.DriveConstants.WHEEL_RIGHT_ANGLE;
import static frc.robot.Constants.TitanConstants.DriveConstants.wheelDistanceFromCenter;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinetics.KilloughDriveKinematics;
import edu.wpi.first.math.kinetics.KilloughDriveOdometry;
import edu.wpi.first.math.kinetics.KilloughDriveWheelPositions;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Odometry implements Sendable {
  private final KilloughDriveKinematics kinematics;
  private final KilloughDriveOdometry odometry;
  private final AHRS gyro;

  public Odometry() {
    final var wheelDistance = new Translation2d(wheelDistanceFromCenter, 0);
    final var leftWheelPos =
        wheelDistance.rotateBy(new Rotation2d(Math.toRadians(WHEEL_LEFT_ANGLE)));
    final var rightWheelPos =
        wheelDistance.rotateBy(new Rotation2d(Math.toRadians(WHEEL_RIGHT_ANGLE)));
    final var backWheelPos =
        wheelDistance.rotateBy(new Rotation2d(Math.toRadians(WHEEL_BACK_ANGLE)));

    kinematics = new KilloughDriveKinematics(leftWheelPos, rightWheelPos, backWheelPos,
        Math.toRadians(WHEEL_LEFT_ANGLE),
        Math.toRadians(WHEEL_RIGHT_ANGLE),
        Math.toRadians(WHEEL_BACK_ANGLE));
    gyro = new AHRS();
    gyro.reset();

    odometry =
        new KilloughDriveOdometry(kinematics, getGyroAngle(),
            new KilloughDriveWheelPositions());

    registerToShuffleboard();
  }

  /** Shuffleboardへの登録をする */
  private void registerToShuffleboard() {
    SendableRegistry.addChild(this, gyro);
    SendableRegistry.addLW(this, "Odometry");
  }

  public Pose2d update(double leftWheelDistance, double rightWheelDistance,
      double backWheelDistance) {
    return odometry.update(getGyroAngle(),
        new KilloughDriveWheelPositions(leftWheelDistance, rightWheelDistance, backWheelDistance));
  }

  /** ジャイロの現在の角度を取得する */
  private Rotation2d getGyroAngle() {
    return new Rotation2d(Math.toRadians(gyro.getAngle()));
  }

  /**
   * オドメトリの現在の角度を取得する
   *
   * @return 現在の角度 (弧度法)
   */
  public double getRotationDegrees() {
    return getRotation().getDegrees();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPose(pose);
  }

  public CommandBase ResetPoseCommand(Pose2d pose) {
    return Commands.withName("Reset Pose", Commands.runOnce(() -> resetPose(pose)));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Pose X", () -> getPose().getTranslation().getX(),
        null);
    builder.addDoubleProperty("Pose Y", () -> getPose().getTranslation().getY(),
        null);
    builder.addDoubleProperty("Angle", () -> getPose().getRotation().getDegrees(),
        null);
  }
}
