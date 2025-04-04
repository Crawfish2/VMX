package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TitanKilloughDrive;

public class DriveDistance extends CommandBase {
  private static final TitanKilloughDrive drive = RobotContainer.drive;
  private final double speed = 0.3;
  private final double angle;
  private final double distance;

  public DriveDistance(double angle, double distance) {
    this.angle = angle;
    this.distance = distance;
    addRequirements(drive);
    drive.resetEncodersDistance();
  }

  @Override
  public void execute() {
    drive.drivePolar(speed, angle, 0);
  }

  @Override
  public void end(boolean interrupted) {
    drive.resetEncodersDistance();
  }

  @Override
  public boolean isFinished() {
    return drive.getDistancePolar(angle) > distance;
  }
}
