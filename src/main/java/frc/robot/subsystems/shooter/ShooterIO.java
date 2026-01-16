package frc.robot.subsystems.shooter;

/**
 * Hardware abstraction for Shooter subsystem.
 * Supports real hardware, simulation, and AdvantageKit logging.
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

    /** Update sensor readings */
    default void updateInputs(ShooterIOInputs inputs) {}

    /** Closed-loop shooter velocity */
    default void setShooterVelocity(double velocityRPM, double ffVolts) {}

    /** Open-loop voltage control for shooter */
    default void setShooterVoltage(double volts) {}

    /** Open-loop voltage control for feeder */
    default void setFeederVoltage(double volts) {}
}
