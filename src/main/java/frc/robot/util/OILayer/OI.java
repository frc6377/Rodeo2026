package frc.robot.util.OILayer;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface OI {
    public final Trigger noButton = new Trigger(() -> false);
    public final DoubleSupplier noAxis = () -> 0.0;

    public final ControlCurve driveTranslationCurve = new ControlCurve(1, 3, 0.2, true);
    public final ControlCurve driveRotationCurve = new ControlCurve(1, 3, 0.2, true);

    default DoubleSupplier driveTranslationX() {
        return noAxis;
    }

    default DoubleSupplier driveTranslationY() {
        return noAxis;
    }

    default DoubleSupplier driveRotation() {
        return noAxis;
    }

    default Trigger zeroDrivebase() {
        return noButton;
    }

    default Trigger button1() {
        return noButton;
    }

    default Trigger button2() {
        return noButton;
    }

    default Trigger button3() {
        return noButton;
    }

    default Trigger button4() {
        return noButton;
    }
}
