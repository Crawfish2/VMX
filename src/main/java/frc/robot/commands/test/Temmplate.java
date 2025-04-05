package frc.robot.commands.test;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Stop;
import frc.robot.subsystems.TitanKilloughDrive;

public class Temmplate extends AutoCommand {
  public Temmplate(TitanKilloughDrive drive) {
    super(
        new Stop(drive).withTimeout(0.1));
  }

}
