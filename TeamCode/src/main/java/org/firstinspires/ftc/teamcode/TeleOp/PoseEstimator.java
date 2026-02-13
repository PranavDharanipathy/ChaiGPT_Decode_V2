package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.util.Rev9AxisImuWrapped;

public class PoseEstimator {

    private Follower follower;
    private Rev9AxisImuWrapped rev9AxisImuWrapped;

    public PoseEstimator(Follower follower, Rev9AxisImuWrapped rev9AxisImuWrapped) {

        this.follower = follower;
        this.rev9AxisImuWrapped = rev9AxisImuWrapped;

    }

    public void update() {

        follower.update();
        rev9AxisImuWrapped.updateVelocities();
    }
}
