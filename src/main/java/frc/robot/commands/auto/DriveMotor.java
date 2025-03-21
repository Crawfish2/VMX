package frc.robot.commands.auto;

// import the SimpleDrive command
import frc.robot.commands.driveCommands.Drive;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class DriveMotor extends AutoCommand {
    /**
     * Constructor
     */
    public DriveMotor(double angle) {
        /**
         * Calls the SimpleDrive command and adds a 5 second timeout
         * When the timeout is complete it will call the end() method in the SimpleDrive
         * command
         */
        super(new Drive(angle).withTimeout(5));
    }
}