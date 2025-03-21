package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;

public class Drive extends CommandBase {
    private static final ExampleSubsystem drive = RobotContainer.drive;
    private double speed;
    private double angle;

    /**
     * Creates a new ExampleCommand.
     *
     * @param angle The angle in degrees.
     */
    public Drive(double angle) {
        this.angle = angle;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        speed = 0.3;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.drive(speed, angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.drive(0.0, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
