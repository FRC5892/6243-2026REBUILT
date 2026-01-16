package frc.robot.subsystems.shooter;

/**
 * Shooter hardware abstraction.
 * Supports:
 * - Real hardware
 * - Simulated hardware
 * - AdvantageKit replay
 */
public interface ShooterIO {

    class ShooterIOInputs {
        public double mainVelocityRPM = 0.0;
        public double followerVelocityRPM = 0.0;
        public double feederVelocityRPM = 0.0;

        public double mainCurrentAmps = 0.0;
        public double followerCurrentAmps = 0.0;
        public double feederCurrentAmps = 0.0;

        public double appliedOutput = 0.0;
        public boolean isFiring = false;
    }

    /** Update all sensor readings */
    default void updateInputs(ShooterIOInputs inputs) {}

    /** Closed-loop shooter control */
    default void setShooterVelocity(double velocityRPM, double ffVolts) {}

    /** Open-loop shooter voltage */
    default void setShooterVoltage(double volts) {}

    /** Feeder control voltage */
    default void setFeederVoltage(double volts) {}
}
