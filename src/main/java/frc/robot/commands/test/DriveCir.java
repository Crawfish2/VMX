package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.Rotate;
import frc.robot.commands.driveCommands.Stop;
import frc.robot.subsystems.TitanKilloughDrive;;

public class DriveCir extends AutoCommand {
  public DriveCir(TitanKilloughDrive drive) {
    super(
        new ParallelCommandGroup(
            new Drive(90, drive).withTimeout(2),
            new Rotate(-0.3, drive).withTimeout(2)),
        new Stop(drive).withTimeout(0.1));
  }
}
