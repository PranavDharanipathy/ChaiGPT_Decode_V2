package org.firstinspires.ftc.teamcode.util.StateMachineUtils;

import java.util.ArrayList;
import java.util.Arrays;

public class StateMachine {

    private Object[] states;
    private Object[] events;

    private Object[][] parseCombinedStateAndEvents(Object[][] stateAndEvents) {

        ArrayList<Object> keys = new ArrayList<>();
        ArrayList<Object> values = new ArrayList<>();

        for (Object[] keyValuePair : stateAndEvents) {

            keys.add(keyValuePair[0]);
            values.add(keyValuePair[1]);
        }

        return new Object[][] {keys.toArray(), values.toArray()};

    }

    //Object should be created using StateMachineFactory
    /// @param statesAndEvents - each argument must be provided in the form of a 2-item array (key-value) pair.
    StateMachine(Object[]... statesAndEvents) {

        Object[][] parsedStatesAndEvents = parseCombinedStateAndEvents(statesAndEvents);

        states = parsedStatesAndEvents[0];
        events = parsedStatesAndEvents[1];
    }

    private Object currentState;

    public void setState(Object state) {

        if (Arrays.stream(states).noneMatch(states -> states.equals(state))) throw new IllegalArgumentException();

        currentState = state;
    }

    public Object getState() {
        return currentState;
    }

    public Object getEvent() {
        return events[Arrays.asList(states).indexOf(currentState)];
    }

}
