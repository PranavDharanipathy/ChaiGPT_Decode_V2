package org.firstinspires.ftc.teamcode.util.StateMachineUtils;

import java.util.ArrayList;
import java.util.List;

public class StateMachineFactory {

    private static final StateMachineFactory stateMachineFactory = new StateMachineFactory();

    public static StateMachineFactory getInstance() {
        return stateMachineFactory;
    }

    public StateMachine createFromInformation(List<Object> states, List<Object> events) {

        verifyListInput(states, events);

        ArrayList<Object[]> combinedStatesAndEvents = new ArrayList<>();

        for (Object state : states) {
            combinedStatesAndEvents.add(new Object[] {state, events.get(states.indexOf(state))});
        }

        Object[][] combinedStatesAndEvents_arrayForm = combinedStatesAndEvents.toArray(new Object[0][]);

        verifyVarargInput(combinedStatesAndEvents_arrayForm);

        return new StateMachine(combinedStatesAndEvents_arrayForm);
    }

    private String[] illegalVarargInput = {"Can only have 2 items in each array inputted with the first item being the key and the second being the value, an array with an illegal length of '", "' was found."};

    private void verifyVarargInput(Object[][] statesAndEvents) {

        for (Object[] info : statesAndEvents) {

            if (info.length != 2) {

                String illegalVarargInputErrorMessage = illegalVarargInput[0] + info.length + illegalVarargInput[1];
                throw new IllegalArgumentException(illegalVarargInputErrorMessage);
            }
        }
    }

    private void verifyListInput(List<Object> states, List<Object> events) {

        if (states.size() != events.size()) {
            throw new IllegalArgumentException("must have same number of 'states' entries as 'events' entries.");
        }
    }

}