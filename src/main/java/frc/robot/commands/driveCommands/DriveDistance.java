package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;

public class DriveDistance extends CommandBase {
  private static final ExampleSubsystem drive = RobotContainer.drive;
  private final double speed = 0.3;
  private final double angle;
  private final double distance;

  public DriveDistance(double angle, double distance) {
    this.angle = angle;
    this.distance = distance;
    addRequirements(drive);
    drive.omniDrive.resetEncodersDistance();
  }

  @Override
  public void execute() {
    drive.omniDrive.move(speed, angle);
  }

  @Override
  public void end(boolean interrupted) {
    drive.omniDrive.resetEncodersDistance();
  }

  @Override
  public boolean isFinished() {
    return drive.omniDrive.getDistance(angle) > distance;
  }
}
