package frc.robot.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class TunableNumber extends SubsystemBase implements DoubleSupplier {
  private static NetworkTableInstance inst;
  private static ShuffleboardTab tuningTab;

  private double value;
  private double defaultValue;
  private Consumer<Double> consumer;
  private DoubleTopic doubleTopic;
  private DoubleSubscriber doubleSub;
  private long latestValue;

  public TunableNumber(String name, double defaultValue, Subsystem subsystem) {
    this(name, defaultValue, (ignored) -> {}, subsystem);
  }

  public TunableNumber(
      String name, double defaultValue, Consumer<Double> consumer, Subsystem subsystem) {
    if (subsystem != null) {
      tuningTab = Shuffleboard.getTab(subsystem.getName());
    } else {
      tuningTab = Shuffleboard.getTab("subsystem");
    }

    this.value = defaultValue;

    this.defaultValue = defaultValue;
    this.consumer = consumer;

    if (true) {
      inst = NetworkTableInstance.getDefault();
      tuningTab.add(name, this.defaultValue);

      doubleTopic = inst.getDoubleTopic(name);
      doubleSub =
          doubleTopic.subscribe(
              this.defaultValue, PubSubOption.pollStorage(2), PubSubOption.periodic(1));
    }
  }

  public void periodic() {
    if (latestValue != doubleSub.getLastChange()) {
      value = doubleSub.get(this.defaultValue);
      consumer.accept(value);
      latestValue = doubleSub.getLastChange();
    }
  }

  public double getAsDouble() {
    return value;
  }
}
