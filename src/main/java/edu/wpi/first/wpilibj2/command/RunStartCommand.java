package edu.wpi.first.wpilibj2.command;

public class RunStartCommand extends CommandBase {
  private final Runnable start;
  private final Runnable run;

  public RunStartCommand(Runnable start, Runnable run, Subsystem... requirements) {
    this.start = start;
    this.run = run;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    start.run();
  }

  @Override
  public void execute() {
    run.run();
  }
}
