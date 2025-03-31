package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.Rotate;
import frc.robot.commands.driveCommands.Stop;;

public class DriveCir extends AutoCommand {
    public DriveCir() {
        super(
                new Rotate(0.3).withTimeout(1.5),
                new ParallelCommandGroup(
                        new Drive(0).withTimeout(2),
                        new Rotate(-0.3).withTimeout(2)),
                new Stop().withTimeout(0.1));
    }
}
