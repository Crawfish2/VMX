package frc.robot.commands.test;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.Stop;
import frc.robot.subsystems.TitanKilloughDrive;

public class DriveSide extends AutoCommand {
  public DriveSide(TitanKilloughDrive drive) {
    super(
        // 右移動
        new Drive(90, drive).withTimeout(1),
        // 左移動
        new Drive(-90, drive).withTimeout(1),

        new Stop(drive).withTimeout(0.1));
  }

}
