package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.StateMachineUtils.StateMachine;
import org.firstinspires.ftc.teamcode.util.StateMachineUtils.StateMachineFactory;

import java.util.Arrays;
import java.util.List;

@TeleOp(group = "testing")
public class StateMachineTesting extends LinearOpMode {

    // JUST AN EXAMPLE

    private StateMachine exampleStateMachine;

    private Object[] states = {"open", "closed"};
    private Object[] events = {"available for pick", "available for drop"};

    @Override
    public void runOpMode() {

        List<Object> states = Arrays.asList(this.states);
        List<Object> events = Arrays.asList(this.events);

        exampleStateMachine = StateMachineFactory.getInstance().createFromInformation(states, events);

        if (isStopRequested()) return;
        waitForStart();

        exampleStateMachine.setState("open");

        telemetry.addData("state", exampleStateMachine.getState());
        telemetry.addData("event", exampleStateMachine.getEvent());
        telemetry.update();

    }
}
