package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;

public class Rotate extends CommandBase {
    private static final ExampleSubsystem drive = RobotContainer.drive;
    private double speed;

    public Rotate(double speed) {
        this.speed = speed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drive.omniDrive.rotate(speed);
    }

    @Override
    public void end(boolean interrupted) {
        drive.omniDrive.move(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
