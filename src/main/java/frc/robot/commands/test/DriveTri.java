package frc.robot.commands.test;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.Rotate;

public class DriveTri extends AutoCommand {
    public DriveTri() {
        super(new Drive(90).withTimeout(2.5),
                new Rotate(0.3).withTimeout(3),
                new Drive(0).withTimeout(10),
                new Rotate(-0.3).withTimeout(3),
                new Drive(0).withTimeout(10)
        );
    }
}
