package frc.robot.commands.test;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.Rotate;
import frc.robot.commands.driveCommands.Stop;

public class Temmplate extends AutoCommand {
    public Temmplate() {
        super(
                new Stop().withTimeout(0.1));
    }

}
