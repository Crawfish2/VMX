package frc.robot.commands.test;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.subsystems.TitanKilloughDrive;

public class Template extends AutoCommand {
  public Template(TitanKilloughDrive drive) {
    // 正面に600mm進む
    super(drive.DriveDistanceCommand(0, 1 * 600));
  }
}
