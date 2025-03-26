package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LambdaCommand extends CommandBase {
    private Runnable onInit;
    private Runnable onExecute;
    private Runnable onEnd;
    private BooleanSupplier isFinished_runner;

    public LambdaCommand(Runnable onInit, Runnable onExecute, Runnable onEnd, BooleanSupplier isFinished_runner,
            Subsystem... requirements) {
        this.onInit = onInit;
        this.onExecute = onExecute;
        this.onEnd = onEnd;
        this.isFinished_runner = isFinished_runner;
        addRequirements(requirements);
    }

    public LambdaCommand(Runnable onInit, Runnable onExecute, Runnable onEnd, Subsystem... requirements) {
        this(onInit, onExecute, onEnd, null, requirements);
    }

    public LambdaCommand(Runnable onExecute, Subsystem... requirements) {
        this(null, onExecute, null, null, requirements);
    }

    @Override
    public void initialize() {
        if (onInit != null)
            onInit.run();
    }

    @Override
    public void execute() {
        if (onExecute != null)
            onExecute.run();
    }

    @Override
    public void end(boolean interrupted) {
        if (onEnd != null)
            onEnd.run();
    }

    @Override
    public boolean isFinished() {
        return isFinished_runner != null && isFinished_runner.getAsBoolean();
    }
}