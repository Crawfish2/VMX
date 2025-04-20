package edu.wpi.first.wpilibj2.command;

import java.util.function.BooleanSupplier;

public class RunDeadlineCommand extends CommandBase {
  private final Runnable run;
  private final BooleanSupplier deadline;

  public RunDeadlineCommand(Runnable run, BooleanSupplier deadline, Subsystem... requirements) {
    this.run = run;
    this.deadline = deadline;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    run.run();
  }

  @Override
  public boolean isFinished() {
    return deadline.getAsBoolean();
  }
}
