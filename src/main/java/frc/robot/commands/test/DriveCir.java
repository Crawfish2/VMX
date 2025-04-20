package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.subsystems.TitanKilloughDrive;;

public class DriveCir extends AutoCommand {
  public DriveCir(TitanKilloughDrive drive) {
    super(
        new ParallelCommandGroup(
            drive.DriveCommand(90).withTimeout(2),
            drive.RotateCommand(-0.3).withTimeout(2)));
  }
}
