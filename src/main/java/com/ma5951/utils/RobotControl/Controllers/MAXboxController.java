package com.ma5951.utils.RobotControl.Controllers;

import edu.wpi.first.wpilibj.XboxController;

/**
 * MAXboxController maps XboxController inputs to the MAController interface.
 */
public class MAXboxController implements MAController {

    private final XboxController controller;

    /**
     * Constructs a new MAXboxController with the given port index.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public MAXboxController(int port) {
        this.controller = new XboxController(port);
    }

    /**
     * Returns true if the left bumper (L1) is pressed.
     */
    @Override
    public boolean getL1() {
        return controller.getLeftBumper();
    }

    /**
     * Returns true if the left trigger (L2) is pressed beyond a threshold.
     */
    @Override
    public boolean getL2() {
        return controller.getLeftTriggerAxis() > 0.1;
    }

    /**
     * Returns true if the right bumper (R1) is pressed.
     */
    @Override
    public boolean getR1() {
        return controller.getRightBumper();
    }

    /**
     * Returns true if the right trigger (R2) is pressed beyond a threshold.
     */
    @Override
    public boolean getR2() {
        return controller.getRightTriggerAxis() > 0.1;
    }

    /**
     * Returns true if the left stick (L3) is pressed.
     */
    @Override
    public boolean getL3() {
        return controller.getLeftStickButton();
    }

    /**
     * Returns true if the right stick (R3) is pressed.
     */
    @Override
    public boolean getR3() {
        return controller.getRightStickButton();
    }

    /**
     * Returns true if the 'Y' button is pressed (mapped as ActionsUp).
     */
    @Override
    public boolean getActionsUp() {
        return controller.getYButton();
    }

    /**
     * Returns true if the 'A' button is pressed (mapped as ActionsDown).
     */
    @Override
    public boolean getActionsDown() {
        return controller.getAButton();
    }

    /**
     * Returns true if the 'X' button is pressed (mapped as ActionsLeft).
     */
    @Override
    public boolean getActionsLeft() {
        return controller.getXButton();
    }

    /**
     * Returns true if the 'B' button is pressed (mapped as ActionsRight).
     */
    @Override
    public boolean getActionsRight() {
        return controller.getBButton();
    }

    /**
     * Returns true if the D-Pad is pressed up.
     */
    @Override
    public boolean getDpadUp() {
        return controller.getPOV() == 0;
    }

    /**
     * Returns true if the D-Pad is pressed down.
     */
    @Override
    public boolean getDpadDown() {
        return controller.getPOV() == 180;
    }

    /**
     * Returns true if the D-Pad is pressed left.
     */
    @Override
    public boolean getDpadLeft() {
        return controller.getPOV() == 270;
    }

    /**
     * Returns true if the D-Pad is pressed right.
     */
    @Override
    public boolean getDpadRight() {
        return controller.getPOV() == 90;
    }

    /**
     * Returns true if the 'Back' button is pressed (mapped as OptionsLeft).
     */
    @Override
    public boolean getOptionsLeft() {
        return controller.getBackButton();
    }

    /**
     * Returns true if the 'Start' button is pressed (mapped as OptionsRight).
     */
    @Override
    public boolean getOptionsRight() {
        return controller.getStartButton();
    }

    /**
     * Dose not work with Xbox controller.
     */
    @Override
    public boolean getMiddle() {
        return false;
    }

    /**
     * Returns the analog value of the right trigger axis (0.0 to 1.0).
     */
    @Override
    public double getRightTrigger() {
        return controller.getRightTriggerAxis();
    }

    /**
     * Returns the analog value of the left trigger axis (0.0 to 1.0).
     */
    @Override
    public double getLeftTrigger() {
        return controller.getLeftTriggerAxis();
    }

    /**
     * Returns the horizontal axis value of the right joystick (-1.0 to 1.0).
     */
    @Override
    public double getRightX() {
        return controller.getRightX();
    }

    /**
     * Returns the vertical axis value of the right joystick (-1.0 to 1.0).
     */
    @Override
    public double getRightY() {
        return controller.getRightY();
    }

    /**
     * Returns the horizontal axis value of the left joystick (-1.0 to 1.0).
     */
    @Override
    public double getLeftX() {
        return controller.getLeftX();
    }

    /**
     * Returns the vertical axis value of the left joystick (-1.0 to 1.0).
     */
    @Override
    public double getLeftY() {
        return controller.getLeftY();
    }
}
