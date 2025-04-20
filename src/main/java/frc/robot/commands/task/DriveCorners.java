package frc.robot.commands.task;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.subsystems.TitanKilloughDrive;

public class DriveCorners extends AutoCommand {
  public DriveCorners(TitanKilloughDrive drive) {
    super(drive.DriveCommand(90).withTimeout(2.5),
        drive.DriveCommand(0).withTimeout(10),
        drive.DriveCommand(-90).withTimeout(5),
        drive.RotateDistanceCommand(180),
        drive.DriveCommand(0).withTimeout(10),
        drive.RotateDistanceCommand(90),
        drive.DriveCommand(0).withTimeout(2.5));
  }
}
