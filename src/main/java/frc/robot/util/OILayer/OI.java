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

    default Trigger salvageIntake() {
        return noButton;
    }

    default Trigger salvageOuttake() {
        return noButton;
    }

    default Trigger scrapIntake() {
        return noButton;
    }

    default Trigger scrapOuttake() {
        return noButton;
    }

    default Trigger pivotStow() {
        return noButton;
    }

    default Trigger pivotScore() {
        return noButton;
    }

    default Trigger test() {
        return noButton;
    }
}
