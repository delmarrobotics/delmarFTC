package test.unused;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
*
*/
public class Buttons {
    public class States {
        boolean down = false;
        boolean pressed = false;
        boolean up = false;
    }
    States a;
    States b;
    States x;
    States y;

    public static Buttons buttons;

    private void getState (boolean button, States state) {
        if (button) {
            if (!state.down && !buttons.a.pressed) {
                state.down = true;
            } else if (state.down) {
                state.down = false;
                state.pressed = true;
            }
        }  else if (state.down) {
            state.down = false;
            state.up = true;
        } else if (state.pressed) {
            state.pressed = false;
            state.up = true;
        } else if (state.up) {
            state.up = false;
        }
    }

    public Buttons getStates (Gamepad gamepad) {

        getState(gamepad.a, buttons.a);
        getState(gamepad.b, buttons.b);
        getState(gamepad.x, buttons.x);
        getState(gamepad.y, buttons.y);

        return buttons;
    }
}
