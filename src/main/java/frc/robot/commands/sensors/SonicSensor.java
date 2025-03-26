package frc.robot.commands.sensors;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SonicSensor extends CommandBase {
    private Ultrasonic sonor;
    private final double deadline;

    public SonicSensor(double deadline) {
        this.deadline = deadline;
    }

    @Override
    public void initialize() {
        sonor = new Ultrasonic(8, 9);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        double distance = sonor.getRangeMM();
        return distance < this.deadline; // デッドラインより壁などと距離が近いと終わる
    }
}
