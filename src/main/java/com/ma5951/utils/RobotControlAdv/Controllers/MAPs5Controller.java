package com.ma5951.utils.RobotControlAdv.Controllers;

import edu.wpi.first.wpilibj.PS5Controller;

/**
 * MAXboxPS5Controller implements MAController for a PlayStation 5 controller.
 */
public class MAPs5Controller implements MAController {

    private final PS5Controller controller;

    /**
     * Constructs a new MAXboxPS5Controller.
     *
     * @param port The port index on the Driver Station where the PS5 controller is connected.
     */
    public MAPs5Controller(int port) {
        this.controller = new PS5Controller(port);
    }

    /** Returns true if L1 is pressed. */
    @Override
    public boolean getL1() {
        return controller.getL1Button();
    }

    /** Returns true if L2 (analog trigger) is pressed beyond a threshold. */
    @Override
    public boolean getL2() {
        return controller.getL2Axis() > 0.1;
    }

    /** Returns true if R1 is pressed. */
    @Override
    public boolean getR1() {
        return controller.getR1Button();
    }

    /** Returns true if R2 (analog trigger) is pressed beyond a threshold. */
    @Override
    public boolean getR2() {
        return controller.getR2Axis() > 0.1;
    }

    /** Returns true if L3 (left stick button) is pressed. */
    @Override
    public boolean getL3() {
        return controller.getL3Button();
    }

    /** Returns true if R3 (right stick button) is pressed. */
    @Override
    public boolean getR3() {
        return controller.getR3Button();
    }

    /** Returns true if the Triangle (△) button is pressed (mapped as ActionsUp). */
    @Override
    public boolean getActionsUp() {
        return controller.getTriangleButton();
    }

    /** Returns true if the Cross (✕) button is pressed (mapped as ActionsDown). */
    @Override
    public boolean getActionsDown() {
        return controller.getCrossButton();
    }

    /** Returns true if the Square (◁) button is pressed (mapped as ActionsLeft). */
    @Override
    public boolean getActionsLeft() {
        return controller.getSquareButton();
    }

    /** Returns true if the Circle (○) button is pressed (mapped as ActionsRight). */
    @Override
    public boolean getActionsRight() {
        return controller.getCircleButton();
    }

    /** Returns true if the D-pad is pressed up. */
    @Override
    public boolean getDpadUp() {
        return controller.getPOV() == 0;
    }

    /** Returns true if the D-pad is pressed down. */
    @Override
    public boolean getDpadDown() {
        return controller.getPOV() == 180;
    }

    /** Returns true if the D-pad is pressed left. */
    @Override
    public boolean getDpadLeft() {
        return controller.getPOV() == 270;
    }

    /** Returns true if the D-pad is pressed right. */
    @Override
    public boolean getDpadRight() {
        return controller.getPOV() == 90;
    }

    /** Returns true if the Create (Back) button is pressed (mapped as OptionsLeft). */
    @Override
    public boolean getOptionsLeft() {
        return controller.getCreateButton();
    }

    /** Returns true if the Options (Start) button is pressed (mapped as OptionsRight). */
    @Override
    public boolean getOptionsRight() {
        return controller.getOptionsButton();
    }

    /** Returns true if the touch button is pressed.*/
    @Override
    public boolean getMiddle() {
        return controller.getTouchpadButton();
    }

    /** Returns the value of the right trigger (R2) axis (0.0 to 1.0). */
    @Override
    public double getRightTrigger() {
        return controller.getR2Axis();
    }

    /** Returns the value of the left trigger (L2) axis (0.0 to 1.0). */
    @Override
    public double getLeftTrigger() {
        return controller.getL2Axis();
    }

    /** Returns the horizontal axis of the right joystick (-1.0 to 1.0). */
    @Override
    public double getRightX() {
        return controller.getRightX();
    }

    /** Returns the vertical axis of the right joystick (-1.0 to 1.0). */
    @Override
    public double getRightY() {
        return controller.getRightY();
    }

    /** Returns the horizontal axis of the left joystick (-1.0 to 1.0). */
    @Override
    public double getLeftX() {
        return controller.getLeftX();
    }

    /** Returns the vertical axis of the left joystick (-1.0 to 1.0). */
    @Override
    public double getLeftY() {
        return controller.getLeftY();
    }
}
