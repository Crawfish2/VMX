package edu.wpi.first.wpilibj2.command;


public class RunEndCommand extends CommandBase {
  private final Runnable run;
  private final Runnable end;

  public RunEndCommand(Runnable run, Runnable end, Subsystem... requirements) {
    this.run = run;
    this.end = end;
    addRequirements(requirements);
  }

  @Override
  public void execute() {
    run.run();
  }

  @Override
  public void end(boolean interrupted) {
    end.run();
  }
}
