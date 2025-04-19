package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TitanKilloughDrive;

public class DriveDistance extends CommandBase {
  private final TitanKilloughDrive drive;
  private final double speed = 0.3;
  private final double angle;
  private final double distance;

  public DriveDistance(double angle, double distance, TitanKilloughDrive drive) {
    this.angle = angle;
    this.distance = distance;
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.resetEncodersDistance();
  }

  @Override
  public void execute() {
    drive.drivePolar(speed, angle, 0);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return drive.getDistancePolar(angle) > distance;
  }
}
