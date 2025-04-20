package frc.robot.commands.test;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.subsystems.TitanKilloughDrive;

public class DriveSide extends AutoCommand {
  public DriveSide(TitanKilloughDrive drive) {
    super(
        // 右移動
        drive.DriveCommand(90).withTimeout(1),
        // 左移動
        drive.DriveCommand(-90).withTimeout(1));
  }

}
