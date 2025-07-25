package frc.robot.commands;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.SimpleCamera;
import frc.robot.subsystems.TitanKilloughDrive;
import frc.robot.subsystems.UltraSonicSensor;
import frc.robot.subsystems.UltraSonicSensor.UltraSonicPosition;

public class Competitions {
  private final TitanKilloughDrive drive;
  @SuppressWarnings("unused") // 後で作る
  private final SimpleCamera camera;
  private final UltraSonicSensor sonar;

  private static final double maxSpeed = 0.3;

  final double targetForwardDistanceMM = 200;
  final double targetSideDistanceMM = 120;

  public Competitions(TitanKilloughDrive drive, SimpleCamera camera, UltraSonicSensor sonar) {
    this.drive = drive;
    this.camera = camera;
    this.sonar = sonar;

    registerToShuffleboard();
  }

  /** Shuffleboardへの登録をする */
  private void registerToShuffleboard() {
    final var tab = Shuffleboard.getTab("Comp");
    tab.add("PoseCollection Left", PoseCollection(Direction.Left, new Pose2d()));
    tab.add("PoseCollection Right", PoseCollection(Direction.Right, new Pose2d()));

    Function<Direction, CommandBase> moveForwardDistanceCommand =
        (Direction direct) -> Commands.defer(() -> {
          var currentPose = drive.odometry.getPose();
          var targetPose = new Pose2d(currentPose.getTranslation().getX() + 600,
              currentPose.getTranslation().getY(),
              currentPose.getRotation());
          return moveForwardDistanceSensor(targetPose, direct);
        }, drive);
    tab.add("moveForwardDistance Left", moveForwardDistanceCommand.apply(Direction.Left));
    tab.add("moveForwardDistance Right", moveForwardDistanceCommand.apply(Direction.Right));
  }

  public CommandBase moveToPose(double x, double y, double angleDegrees) {
    return drive.moveToPoseCommand(new Pose2d(x, y, Rotation2d.fromDegrees(angleDegrees)),
        drive.odometry::getPose);
  }

  public CommandBase moveForwardDistanceSensor(Pose2d pose, Direction direction) {

    final DoubleSupplier getLateralSpeed =
        Direction.Left.equals(direction)
            ? () -> MathUtil.clamp(
                -(sonar.getRangeMM(UltraSonicPosition.middleLeft)
                    - targetSideDistanceMM) / 200,
                -maxSpeed,
                maxSpeed)
            : () -> MathUtil.clamp(
                (sonar.getRangeMM(UltraSonicPosition.middleRight)
                    - targetSideDistanceMM) / 200,
                -maxSpeed,
                maxSpeed);

    return Commands
        .functional(() -> drive.posDriver.setTarget(pose), () -> {
          var currentPose = drive.odometry.getPose();
          currentPose =
              new Pose2d(currentPose.getTranslation().getX(), pose.getTranslation().getY(),
                  currentPose.getRotation());
          var vel = drive.posDriver.getVelocity(currentPose);
          drive.driveCartesian(getLateralSpeed.getAsDouble(), vel.vx, vel.zRotation);
        }, (interrupted) -> drive.odometry.resetPose(pose), drive.posDriver::isCompleted, drive);
  }

  /**
   * 前方と横の壁を使って蛇行修正をする
   *
   * @param direction 左右どちらか
   * @param pose 修正後の現在位置
   */
  public CommandBase PoseCollection(Direction direction, Pose2d pose) {
    if (!Arrays.stream(new Direction[] {Direction.Left, Direction.Right})
        .anyMatch(direct -> direct.equals(direction))) {
      throw new RuntimeException("蛇行修正に使用する壁に左右どちらかを指定する必要がある");
    }

    final double timeout = 3.0;

    final DoubleSupplier getForwardSpeed =
        () -> MathUtil.clamp((sonar.getForwardAvg() - targetForwardDistanceMM) / 200,
            -maxSpeed, maxSpeed);

    final DoubleSupplier getLateralSpeed =
        Direction.Left.equals(direction)
            ? () -> MathUtil.clamp(
                -(sonar.getRangeMM(UltraSonicPosition.middleLeft) - targetSideDistanceMM) / 200,
                -maxSpeed,
                maxSpeed)
            : () -> MathUtil.clamp(
                (sonar.getRangeMM(UltraSonicPosition.middleRight) - targetSideDistanceMM) / 200,
                -maxSpeed,
                maxSpeed);

    final DoubleSupplier getRotate =
        () -> MathUtil.clamp(sonar.getForwardDiff() / 100, -0.15, 0.15);

    return Commands.run(
        () -> drive.driveCartesian(getLateralSpeed.getAsDouble(), getForwardSpeed.getAsDouble(),
            getRotate.getAsDouble()),
        drive, sonar)
        .withTimeout(timeout)
        .andThen(drive.odometry.ResetPoseCommand(pose));
  }

  public CommandBase CollectForward(Pose2d pose) {
    final double timeout = 3.0;

    final DoubleSupplier getRotate =
        () -> MathUtil.clamp(sonar.getForwardDiff() / 100, -0.15, 0.15);
    final DoubleSupplier getForwardSpeed =
        () -> MathUtil.clamp((sonar.getForwardAvg() - targetForwardDistanceMM) / 200,
            -maxSpeed, maxSpeed);
    return new SequentialCommandGroup(
        Commands.run(
            () -> drive.driveCartesian(0, 0,
                getRotate.getAsDouble()),
            drive)
            .withTimeout(timeout),
        Commands.run(
            () -> drive.driveCartesian(0, getForwardSpeed.getAsDouble(), 0), drive))
                .withTimeout(timeout);
  }

  public static enum Direction {
    Forward, Back, Right, Left
  };
}
