package org.firstinspires.ftc.teamcode.ShooterSystems;

public class MotionProfiling {


    //make function that take targetposition, and set a variable to tARGETPOSITION, then creat another function to set an incrementt
    double target;
    double increment;

    public void setTarget(double target) {

        this.target = target;

    }

    public void increment(double increment) {
        this.increment = increment;
    }


    public enum profile_state {

        CLIMBING, PLATEAU
    }

    public profile_state state;

    double realTurretTargetPos;

    double dT;
    double currentPos;

    public void update() {
        if (Math.abs(target - realTurretTargetPos) < increment) {

            state = profile_state.PLATEAU;

        }

        else {
            state = profile_state.CLIMBING;
        }

        if (state == profile_state.CLIMBING) {
            double thisLoopInc = (target > currentPos) ? increment : -increment;

            realTurretTargetPos += thisLoopInc;
        }
        else {
            realTurretTargetPos = target;
        }

    }



}
