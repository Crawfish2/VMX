package frc.robot.commands.task;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.Rotate;

public class DriveCorners extends AutoCommand {
  public DriveCorners() {
    super(new Drive(90).withTimeout(2.5),
        new Drive(0).withTimeout(10),
        new Drive(-90).withTimeout(5),
        new Rotate(0.3).withTimeout(5),
        new Drive(0).withTimeout(10),
        new Rotate(0.3).withTimeout(2.5),
        new Drive(0).withTimeout(2.5));
  }
}
