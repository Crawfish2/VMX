package frc.robot.commands.test;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.subsystems.TitanKilloughDrive;

public class DriveTri extends AutoCommand {
  public DriveTri(TitanKilloughDrive drive) {
    super(drive.DriveDistanceCommand(0, 300),
        drive.RotateDistanceCommand(60),
        drive.DriveDistanceCommand(0, 300),
        drive.RotateDistanceCommand(60),
        drive.DriveDistanceCommand(0, 300),
        drive.RotateDistanceCommand(60));
  }
}
