package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Full shooter subsystem logic:
 * - AdvantageKit logging
 * - Characterization hooks
 * - Idle / firing / closed-loop velocity control
 * - Simulation support
 */
public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    /** Closed-loop shooter control with optional firing boost */
    public void setShooterSpeed(double velocityRPM, boolean firingBoost) {
        if (Math.abs(velocityRPM) < 1.0) {
            io.setShooterVoltage(0.0); // SPEED LIMIT: deadband
            return;
        }
        double ffVolts = firingBoost ? ShooterConstants.kFiringBoost.get() : 0.0; // SPEED LIMIT
        io.setShooterVelocity(velocityRPM, ffVolts);
    }

    /** Spin shooter at idle voltage */
    public void setIdle() {
        io.setShooterVoltage(ShooterConstants.kIdleVoltage.get()); // SPEED LIMIT
        setFiring(false);
    }

    /** Fire feeder */
    public void setFiring(boolean firing) {
        io.setFeederVoltage(firing ? ShooterConstants.kFeederVoltage.get() : 0.0); // SPEED LIMIT
    }

    /** Check if shooter is at target velocity */
    public boolean isAtSpeed(double targetRPM) {
        return Math.abs(inputs.mainVelocityRPM - targetRPM) < ShooterConstants.kVelocityTolerance.get(); // SPEED LIMIT
    }

    /** Run a voltage sweep for characterization */
    public void runCharacterization(double testVoltage) {
        io.setShooterVoltage(testVoltage); // AdvantageKit logs automatically
    }

    /** Get current shooter velocity */
    public double getShooterVelocity() {
        return inputs.mainVelocityRPM;
    }

    /** Get current drawn by feeder */
    public double getFeederCurrent() {
        return inputs.feederCurrentAmps;
    }
}
