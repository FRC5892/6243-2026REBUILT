package frc.robot.subsystems.shooter.flywheel;

import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Real implementation of FlywheelIO using your team’s LoggedTalonFX. */
public class FlywheelIOReal implements FlywheelIO {
    private final LoggedTalonFX motor;

    public FlywheelIOReal(int motorId) {
        // construct the TalonFX with PID tuning enabled
        motor = Constants.currentMode == Constants.Mode.REAL
            ? new HardwareTalonFX(motorId, "Flywheel")
                  .withPIDTunable(Constants.Flywheel.talonConfig)
            : null; // fallback just in case
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        if (motor == null) return;
        inputs.velocityRadsPerSec = motor.getVelocity().get(RadiansPerSecond);
        inputs.appliedVolts = motor.getPrimaryAppliedVoltage().get(Volts);
    }

    @Override
    public void applyOutputs(FlywheelIOOutputs outputs) {
        if (motor == null) return;
        // only implementing the bang-bang / velocity mode
        motor.setControl(
            motor.getPrimaryVelocityControlRequest(outputs.velocityRadsPerSec)
        );
    }

    public LoggedTalonFX getMotor() {
        return motor;
    }
}
