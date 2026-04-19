package frc.robot.util.LoggedTalon.TalonFX;

import com.ctre.phoenix6.CANBus;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;
import frc.robot.util.LoggedTalon.TalonInputs;
import jsim.PhysicsBody;
import jsim.PhysicsWorld;
import jsim.Vec3;

public class TalonFXSimpleMotorSim extends BaseTalonFXSim {
  // JSim does not yet support mechanism-level motor simulation in Java.
  // This stub uses a PhysicsBody to represent the motor shaft as a rigid body.
  private static final PhysicsWorld world =
      new PhysicsWorld(0.02, false); // Shared world for all sim motors
  private final PhysicsBody shaftBody;

  /**
   * A simple motor sim representing a {@link LoggedTalonFX}
   *
   * <p>This sim is enhanced with CTRE's high fidelity simulation
   *
   * @param canID The motor's CAN ID
   * @param canBus The motor's CAN Bus
   * @param name The Motors Name. This <strong>MUST NOT</strong> be changed in replay.
   * @param J_KgMetersSquared The inertia of the system, in Kgm^2. See {@link
   *     LinearSystemId#createDCMotorSystem(DCMotor, double, double)}
   * @param gearReduction The gear reduction of the system. See {@link
   *     LinearSystemId#createDCMotorSystem(DCMotor, double, double)}
   * @param followers Followers, if any. Followers will share the same output as the leader. All
   *     followers are designed to be physically connected to the leader and as such their velocity
   *     and position are not accessible separately. The current number off followers
   *     <strong>MUST</strong> be passed into simulation and replay.
   */
  public TalonFXSimpleMotorSim(
      int canID,
      CANBus canBus,
      String name,
      double J_KgMetersSquared,
      double gearReduction,
      PhoenixTalonFollower... followers) {
    super(canID, canBus, name, followers);
    // Create a 1kg shaft body at origin (customize as needed)
    this.shaftBody = world.createBody(1.0);
    shaftBody.setPosition(new Vec3(0.0, 0.0, 0.0));
    shaftBody.setLinearVelocity(new Vec3(0.0, 0.0, 0.0));
  }

  @Override
  protected void simulationPeriodic(TalonInputs inputs) {
    // Example: set shaft velocity based on input voltage (not physically accurate)
    double voltage = motorSimState.getMotorVoltage();
    // For now, just map voltage to a linear velocity for demonstration
    shaftBody.setLinearVelocity(new Vec3(voltage, 0.0, 0.0));
    world.step();

    // Use shaftBody's X position and velocity as stand-ins for rotor position/velocity
    motorSimState.setRotorVelocity(shaftBody.linearVelocity().x());
    motorSimState.setRawRotorPosition(shaftBody.position().x());
    // NOTE: This is a placeholder. Replace with real mechanism simulation when JSim exposes it in
    // Java.
  }
}
