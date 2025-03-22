package frc.robot.util;

import java.util.Arrays;
import java.util.stream.IntStream;

import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import frc.robot.Constants;

public class OmniDrive {
    private TitanQuad[] motors;
    private TitanQuadEncoder[] motors_enc;

    public OmniDrive() {
        motors = IntStream.range(0, 3).mapToObj(i -> new TitanQuad(Constants.TITAN_ID, i)).toArray(TitanQuad[]::new);
        motors_enc = IntStream.range(0, 3)
                .mapToObj(i -> new TitanQuadEncoder(motors[i], i, Constants.WHEEL_DIST_PER_TICK))
                .toArray(TitanQuadEncoder[]::new);
        Arrays.stream(motors_enc).forEach(motor_enc -> motor_enc.reset());
    }

    /**
     * Will give the distance traveled based on encoder data
     *
     * @param motor Motor number
     * @return Distance traveled
     */
    public double getEncoderDistance(int motor) {
        return motors_enc[motor].getEncoderDistance();
    }

    /**
     * Gets limit switch status for specified motor
     *
     * @param motor     Motor number
     * @param direction Direction (true: low side, false: high side)
     * @return Limit switch value
     */
    public double getLimitSwitch(int motor, boolean direction) {
        return motors[motor].getLimitSwitch(motor, direction);
    }

    private void setMotorsSpeed(double[] speeds) {
        for (int i = 0; i < 3; i++) {
            motors[i].set(speeds[i]);
        }
    }

    /**
     * Moves the robot with specified speed and angle
     *
     * @param speed The speed to move (range -1.0 to 1.0)
     * @param angle The angle in degrees (0 degrees is forward)
     */
    public void move(double speed, double angle) {
        double[] motorsSpeed = new double[3];
        motorsSpeed[0] = Math.sin(Math.toRadians(angle + 180));
        motorsSpeed[1] = Math.sin(Math.toRadians(angle - 60));
        motorsSpeed[2] = Math.sin(Math.toRadians(angle + 60));

        double x = Arrays.stream(motorsSpeed).map(s -> Math.abs(s)).max().getAsDouble();

        // Normalize
        motorsSpeed[0] = motorsSpeed[0] * speed / x;
        motorsSpeed[1] = motorsSpeed[1] * speed / x;
        motorsSpeed[2] = motorsSpeed[2] * speed / x;

        setMotorsSpeed(motorsSpeed);
    }

    /**
     * Rotates the robot with specified speed
     *
     * @param speed The speed to rotate (range -1.0 to 1.0)
     */
    public void rotate(double speed) {
        double[] motorsSpeed = new double[3];
        motorsSpeed[0] = speed;
        motorsSpeed[1] = speed;
        motorsSpeed[2] = speed;

        setMotorsSpeed(motorsSpeed);
    }
}
