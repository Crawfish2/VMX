package frc.robot.commands.test;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.Stop;

public class DriveSide extends AutoCommand {
    public DriveSide() {
        super(
                new Drive(90).withTimeout(1),
                new Drive(-90).withTimeout(1),
                new Stop().withTimeout(0.1));
    }

}
