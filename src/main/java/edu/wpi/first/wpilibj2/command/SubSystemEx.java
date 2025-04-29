package edu.wpi.first.wpilibj2.command;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import javax.annotation.Nullable;

// 元のドキュメント: Original document:
// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Subsystem.html

public interface SubSystemEx extends Subsystem {
  /**
   * Constructs a command that runs an action every iteration until interrupted.
   *
   * @param action - the action to run
   * @see RunCommand
   */
  default CommandBase run(Runnable action) {
    return Commands.run(action, this);
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted, and then runs a
   * second action.
   *
   * @param run - the action to run every iteration
   * @param end - the action to run on interrupt
   * @see RunEndCommand
   */
  default CommandBase runEnd(Runnable run, Runnable end) {
    return Commands.runEnd(run, end, this);
  }

  /**
   * Constructs a command that runs an action once and finishes.
   *
   * @param action - the action to run
   * @see InstantCommand
   */
  default CommandBase runOnce(Runnable action) {
    return Commands.runOnce(action, this);
  }

  /**
   * Constructs a command that runs an action once and another action when the command is
   * interrupted.
   *
   * @param start - the action to run on start
   * @param end - the action to run on interrupt
   * @see StartEndCommand
   */
  default CommandBase startEnd(Runnable start, Runnable end) {
    return Commands.startEnd(start, end, this);
  }

  /**
   * Constructs a command that runs an action once and then runs another action every iteration
   * until interrupted.
   *
   * @param start - the action to run on start
   * @param run - the action to run every iteration
   * @see RunStartCommand
   */
  default CommandBase startRun(Runnable start, Runnable run) {
    return Commands.startRun(start, run);
  }

  /**
   * Constructs a DeferredCommand with the provided supplier. This subsystem is added as a
   * requirement.
   *
   * @param supplier - the command supplier.
   * @see DeferredCommand
   */
  default CommandBase defer(Supplier<Command> supplier) {
    return Commands.defer(supplier, this);
  }

  // 追加のメソッド

  /**
   * 開始時、定期実行時、終了時、終了条件を元にコマンドを作成する
   * 何も実行したくないところは、nullを入れてもよい
   *
   * @param start - 開始時のアクション
   * @param run - 実行中のアクション
   * @param end - 終了時のアクション
   * @param isFinished - 終了条件
   * @see FunctionalCommand
   */
  default CommandBase functional(@Nullable Runnable start, @Nullable Runnable run,
      @Nullable Consumer<Boolean> end,
      @Nullable BooleanSupplier isFinished) {
    return Commands.functional(start, run, end, isFinished, this);
  }

  default CommandBase runDeadline(Runnable run, BooleanSupplier isFinished) {
    return Commands.runDeadline(run, isFinished, this);
  }

  default <T extends CommandBase> T withName(String name, T command) {
    return Commands.withName(name, command);
  }
}
