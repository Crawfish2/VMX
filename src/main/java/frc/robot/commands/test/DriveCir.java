package frc.robot.commands.test;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.subsystems.TitanKilloughDrive;;

public class DriveCir extends AutoCommand {
  public DriveCir(TitanKilloughDrive drive) {
    // 前進しつつ、時計回りに回転する
    super(drive.DrivePolarCommand(0.3, 90, 0.3).withTimeout(2));
  }
}
