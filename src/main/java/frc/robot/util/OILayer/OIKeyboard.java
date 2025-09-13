package frc.robot.util.OILayer;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class OIKeyboard implements OI {
    private static final XboxController controller = new XboxController(0);

    public static final DoubleSupplier LeftX = () -> controller.getRawAxis(0);
    public static final DoubleSupplier LeftY = () -> controller.getRawAxis(1);
    public static final DoubleSupplier RightX = () -> controller.getRawAxis(2);
    public static final DoubleSupplier RightY = () -> controller.getRawAxis(3);

    public static final Trigger Q = new JoystickButton(controller, 1);
    public static final Trigger E = new JoystickButton(controller, 2);
    public static final Trigger R = new JoystickButton(controller, 3);
    public static final Trigger T = new JoystickButton(controller, 4);
    public static final Trigger Y = new JoystickButton(controller, 5);
    public static final Trigger U = new JoystickButton(controller, 6);
    public static final Trigger I = new JoystickButton(controller, 7);
    public static final Trigger O = new JoystickButton(controller, 8);
    public static final Trigger P = new JoystickButton(controller, 9);
    public static final Trigger F = new JoystickButton(controller, 10);
    public static final Trigger G = new JoystickButton(controller, 11);
    public static final Trigger H = new JoystickButton(controller, 12);
    public static final Trigger J = new JoystickButton(controller, 13);
    public static final Trigger K = new JoystickButton(controller, 14);
    public static final Trigger L = new JoystickButton(controller, 15);
    public static final Trigger Z = new JoystickButton(controller, 16);
    public static final Trigger X = new JoystickButton(controller, 17);
    public static final Trigger C = new JoystickButton(controller, 18);
    public static final Trigger V = new JoystickButton(controller, 19);
    public static final Trigger B = new JoystickButton(controller, 20);
    public static final Trigger N = new JoystickButton(controller, 21);
    public static final Trigger M = new JoystickButton(controller, 22);
    public static final Trigger LeftBracket = new JoystickButton(controller, 23);
    public static final Trigger Apostrophe = new JoystickButton(controller, 24);
    public static final Trigger Comma = new JoystickButton(controller, 25);
    public static final Trigger Period = new JoystickButton(controller, 26);
    public static final Trigger ForwardSlash = new JoystickButton(controller, 27);

    @Override
    public DoubleSupplier driveTranslationX() {
        return LeftX;
    }

    @Override
    public DoubleSupplier driveTranslationY() {
        return LeftY;
    }

    @Override
    public DoubleSupplier driveRotation() {
        return RightX;
    }
}
