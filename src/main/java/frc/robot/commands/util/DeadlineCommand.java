package frc.robot.commands.util;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DeadlineCommand extends CommandBase {
  private final BooleanSupplier deadline;

  public DeadlineCommand(BooleanSupplier deadline, Subsystem... requirements) {
    this.deadline = deadline;
    addRequirements(requirements);
  }

  @Override
  public boolean isFinished() {
    return deadline.getAsBoolean();
  }
}
