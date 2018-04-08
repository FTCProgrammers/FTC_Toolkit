package org.firstinspires.ftc.toolkit.Utilities;

public class Toggle {
    private double cooldownTime;
    private boolean toggled;
    private double time;

    public Toggle(double cooldownTime) {
        this.cooldownTime = cooldownTime;
        toggled = false;
        time = System.nanoTime();
    }

    public Toggle() {
        cooldownTime = 0.25;
        toggled = false;
        time = System.nanoTime();
    }

    public void toggle() {
        if((System.nanoTime() - time) / 1e9 >= cooldownTime) {
            toggled = !toggled;
            time = System.nanoTime();
        }
    }

    public void setToggle(boolean toggled) {
        this.toggled = toggled;
    }

    public boolean isToggled() {
        return toggled;
    }
}
