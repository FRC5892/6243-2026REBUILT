package frc.robot.util.LoggedDIO;

public class NoOppDio extends LoggedDIO {
  public NoOppDio(String name) {
    super(name);
  }

  @Override
  public void updateInputs(DIOInputsAutoLogged inputs) {}
}
