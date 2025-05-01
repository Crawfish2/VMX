package frc.robot.commands;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.SimpleCamera;
import frc.robot.subsystems.TitanKilloughDrive;
import frc.robot.subsystems.UltraSonicSensor;
import frc.robot.subsystems.UltraSonicSensor.UltraSonicPosition;

public class Competitions {
  private final TitanKilloughDrive drive;
  private final SimpleCamera camera;
  private final UltraSonicSensor sonor;

  public Competitions(TitanKilloughDrive drive, SimpleCamera camera, UltraSonicSensor sonor) {
    this.drive = drive;
    this.camera = camera;
    this.sonor = sonor;
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
    final double maxSpeed = 0.5;
    final double targetDistanceMM = 10;

    final DoubleSupplier getForwardSpeed =
        () -> MathUtil.clamp((sonor.getForwardAvg() - targetDistanceMM) / 50,
            -maxSpeed, maxSpeed);

    final DoubleSupplier getLateralSpeed =
        Direction.Left.equals(direction)
            ? () -> MathUtil.clamp(
                -(sonor.getRangeMM(UltraSonicPosition.middleLeft) - targetDistanceMM) / 50,
                -maxSpeed,
                maxSpeed)
            : () -> MathUtil.clamp(
                (sonor.getRangeMM(UltraSonicPosition.middleRight) - targetDistanceMM) / 50,
                -maxSpeed,
                maxSpeed);

    final DoubleSupplier getRotate =
        () -> MathUtil.clamp(sonor.getForwardDiff() / 30, -maxSpeed, maxSpeed);

    return Commands.run(
        () -> drive.driveCartesian(getLateralSpeed.getAsDouble(), getForwardSpeed.getAsDouble(),
            getRotate.getAsDouble()),
        drive, sonor)
        .withTimeout(timeout)
        .andThen(drive.odometry.ResetPoseCommand(pose));
  }

  public static enum Direction {
    Forward, Back, Right, Left
  };
}
