package frc.robot.commands.task;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.subsystems.SimpleCamera;
import frc.robot.subsystems.TitanKilloughDrive;
import frc.robot.subsystems.SimpleCamera.ColorType;

/**
 * 課題1
 *
 * 課題のレイアウトは以下の通り
 * <pre>
 * {-, |} : 壁
 * {.} : 黒線
 * {x} : 指示板
 * {->} : ロボットの開始位置、向き
 * {R, B, Y} : 赤、青、黄エリア
 * </pre>
 * @formatter:off
 * <pre>
 * +----+----+----+
 * |RRRR.BBBB|YYYY|
 * |RRRR.BBBB|YYYY|
 * |....-----+....|
 * |    .    |    |
 * |    .    |    |
 * |----.    |    |
 * |    .    .    |
 * |    .    .    |
 * |    .    .    |
 * |..............|
 * |    . -> . xx |
 * |    . -> . xx |
 * +----+----+----+
 * </pre>
 * @formatter:on
 */
public class PubKadai1 extends AutoCommand {
  public PubKadai1(TitanKilloughDrive drive, SimpleCamera camera) {

    // 課題
    // TODO: テストする
    super(
        camera.DetectColorCommand(),
        drive.ResetEncodersDistanceCommand(),
        drive.RotateDistanceCommand(-90),
        new ConditionalCommand(
            // 赤、青のとき
            sequence(
                drive.DriveDistanceCommand(0, 2.5 * 600),
                drive.DriveDistanceCommand(-90, 1 * 600),
                drive.DriveDistanceCommand(0, 1 * 600),
                new ConditionalCommand(
                    // 青のとき
                    drive.DriveDistanceCommand(90, 1 * 600),
                    // 赤のとき
                    new InstantCommand(), // 何もしない
                    () -> camera.getDetectedColor().equals(ColorType.BLUE))),

            // 黄のとき
            sequence(
                drive.DriveDistanceCommand(0, 1 * 600),
                drive.DriveDistanceCommand(90, 1 * 600),
                drive.DriveDistanceCommand(0, 1 * 600)),

            () -> !camera.getDetectedColor().equals(ColorType.YELLOW)));
  }
}
