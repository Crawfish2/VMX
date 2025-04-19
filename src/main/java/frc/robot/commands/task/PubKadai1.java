package frc.robot.commands.task;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.DriveDistance;
import frc.robot.subsystems.TitanKilloughDrive;

/** 課題1 */
public class PubKadai1 extends AutoCommand {
  public PubKadai1(TitanKilloughDrive drive) {
    // 未完成 色指示板を読まずに、赤エリアへ移動する
    super(drive.RotateDistanceCommand(-90),
        new DriveDistance(0, 2.5 * 600, drive),
        new DriveDistance(90, 1 * 600, drive),
        new DriveDistance(0, 1 * 600, drive));
  }
}
