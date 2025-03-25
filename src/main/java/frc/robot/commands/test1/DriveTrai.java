package frc.robot.commands.test1;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.Rotate;

public class DriveTrai extends AutoCommand {
    public DriveTrai() {
        super(new Drive(90).withTimeout(2.5),
                new Drive(0).withTimeout(10),
                new Drive(-90).withTimeout(5),
                new Rotate(0.3).withTimeout(5),
                new Drive(0).withTimeout(10),
                new Rotate(0.3).withTimeout(2.5),
                new Drive(0).withTimeout(2.5)
        );
    }
}
