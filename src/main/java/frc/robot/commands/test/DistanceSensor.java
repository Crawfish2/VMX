package frc.robot.commands.test;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.Rotate;
import frc.robot.subsystems.TitanKilloughDrive;

public class DistanceSensor extends AutoCommand {
  public DistanceSensor(TitanKilloughDrive drive) {
    super(new Drive(90, drive).withTimeout(2.5),
        new Rotate(0.3, drive).withTimeout(2.5),
        new Drive(0, drive).withTimeout(10),
        new Rotate(-0.3, drive).withTimeout(2.5),
        new Drive(0, drive).withTimeout(10),
        new Rotate(0, drive).withTimeout(1));
    // new ConditionalCommand(new Drive(0).withTimeout(0.1), new Drive(0), );
  }

}
