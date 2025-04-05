package frc.robot.commands.test;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.Rotate;
import frc.robot.subsystems.TitanKilloughDrive;

public class DriveTri extends AutoCommand {
  public DriveTri(TitanKilloughDrive drive) {
    super(new Drive(90, drive).withTimeout(2.5),
        new Rotate(-0.3, drive).withTimeout(1.5),
        new Drive(0, drive).withTimeout(3),
        new Rotate(-0.3, drive).withTimeout(1.5),
        new Drive(0, drive).withTimeout(3),
        new Rotate(0.3, drive).withTimeout(3),
        new Rotate(0, drive).withTimeout(1));
  }
}
