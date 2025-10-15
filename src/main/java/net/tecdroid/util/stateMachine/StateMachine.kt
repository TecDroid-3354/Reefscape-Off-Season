package net.tecdroid.util.stateMachine

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import net.tecdroid.util.milliseconds
import net.tecdroid.util.stateMachine.builders.ConditionBuilder
import org.jgrapht.Graph
import org.jgrapht.graph.DefaultDirectedGraph
import org.jgrapht.graph.DefaultEdge
import net.tecdroid.util.stateMachine.builders.ConditionBuilder.Condition


enum class States(var config: StateConfig) {
    MarcoState(StateConfig()),
    AlgaeState(StateConfig()),
    ScoreState(StateConfig()),
    IntakeState(StateConfig()),
    ClimbState(StateConfig());

    /**
     * Change the initial state command. These classes are global, so the config
     * will change for all the instances
     * @param command the command that will execute at the beggining of the state
     */

    fun setInitialCommand(command: Command) {
        this.config.initialCommand = command
    }

    /**
     * Change the default state command. These classes are global, so the config
     * will change for all the instances
     * @param command the command that will execute periodically during of the state
     */

    fun setDefaultCommand(command: Command) {
        this.config.defaultCommand = command
    }

    /**
     * Change the end state command. These classes are global, so the config
     * will change for all the instances
     * @param command the command that will execute at the end of the state
     */

    fun setEndCommand(command: Command) {
        this.config.endCommand = command
    }
}

enum class Phase {
    Teleop,
    Simulation,
    Autonomous,
    All
}

class EdgeCommand : DefaultEdge() {
    var command: Command = Commands.none()
    var restricted : Boolean = false
}

/**
 * The states
 * @param currentState Initial State
 * @param changeStateCommand Command executed when we change the state
 */
class StateMachine(private var currentState: States) : SubsystemBase() {

    private val graph: Graph<States, EdgeCommand> = DefaultDirectedGraph(EdgeCommand::class.java)


    // Change the state default command
    private fun changeStateDefaultCommand(defaultCommand: Command) {
        removeDefaultCommand()
        defaultCommand.addRequirements(this)
        setDefaultCommand(defaultCommand)
    }

    init {
        val initialDefaultCommand : Command = currentState.config.defaultCommand
        initialDefaultCommand.addRequirements(this)

        defaultCommand = initialDefaultCommand

        WaitCommand(0.15.milliseconds).schedule()
        currentState.config.initialCommand.schedule()
    }

    /**
     * Change the state, set the new default command, execute the end command of the last state and
     * execute the change commands (General command executed when we change to another state).
     * @param targetState New state
     */

    fun changeState(targetState: States) {
        if (targetState !=  currentState) {
            transition(targetState)
        }
    }

    /**
     * Change the state, set the new default command, execute the end command of the last state and
     * execute the change commands (General command executed when we change to another state).
     * If the unless parameter is true, the transition won't be executed
     * @param targetState New state
     * @param unless Unless condition to add special cases in which you don't want to change the state
     */

    fun changeState(targetState: States, unless: Boolean) {
        if (targetState !=  currentState && !unless) {
            transition(targetState)
        }
    }

    private fun transition(targetState: States) {
        val edge = graph.getEdge(currentState, targetState)

        // Check if we have a complex relation
        if (edge != null && !edge.restricted) {
            edge.command.schedule()

            // Normal transition

            // execute end command
            currentState.config.endCommand.schedule()

            WaitCommand(0.15.milliseconds).schedule()

            // execute the initial command of the new state
            targetState.config.initialCommand.schedule()

            // set the new default command
            changeStateDefaultCommand(targetState.config.defaultCommand)

            // Change the state
            currentState = targetState
        } else if (edge == null) {
            // Normal transition

            // execute end command
            currentState.config.endCommand.schedule()

            // execute the initial command of the new state
            targetState.config.initialCommand.schedule()

            // set the new default command
            changeStateDefaultCommand(targetState.config.defaultCommand)

            // Change the state
            currentState = targetState
        }

    }

    fun addEdge(from: States, to: States, edgeCommand: Command, restricted : Boolean = false) {
        val edgeCmd = EdgeCommand()
        edgeCmd.command = edgeCommand
        edgeCmd.restricted = restricted

        graph.addVertex(from)
        graph.addVertex(to)

        graph.addEdge(from, to, edgeCmd)
    }

    /**
     * @return The current state
     */
    fun getCurrentState() : States = currentState

    /**
     * Verify if the current state its equal to the param one
     * @param state state to compare
     * @return If the param state its equal to the current one
     */
    fun isState(state: States): () -> Boolean = { currentState == state }

    // Condition system
    val generalConditions = mutableListOf<Condition>()
    val teleopConditions = mutableListOf<Condition>()
    val simulationConditions = mutableListOf<Condition>()
    val autoConditions = mutableListOf<Condition>()

    fun getConditionList(executionPhase: Phase): MutableList<Condition> {
        return when(executionPhase) {
            Phase.Teleop -> teleopConditions
            Phase.Simulation -> simulationConditions
            Phase.Autonomous -> autoConditions
            Phase.All -> generalConditions
        }
    }

    /**
     * Add a condition that if true, it changes the state. This condition is
     * evaluated periodically depending on his phase execution
     * @param condition Condition that change the state
     * @param targetState Target State
     * @param executionPhase Phase in which you will evaluate the condition
     */

    fun addCondition(condition: () -> Boolean, targetState: States, executionPhase: Phase): ConditionBuilder {
        // Create a builder to do the method chaining or DSL
        val builder = ConditionBuilder(this, condition, targetState, executionPhase)
        builder.storeCondition()

        return builder
    }

    private fun assess(conditions : MutableList<Condition>) {
        // Check all the conditions
        for (condition in conditions) {
            // Check if we are not trying to change to the same state
            if (condition.targetState != currentState && condition.condition()) {
                changeState(condition.targetState)
            }
        }
    }

    override fun periodic() {
        // Evaluate each condition depending on his execute phase
        if (DriverStation.isAutonomous()) {
            assess(autoConditions)
        } else if (DriverStation.isTeleop()) {
            assess(teleopConditions)
        } else if (DriverStation.isTest()) {
            assess(simulationConditions)
        }

        assess(generalConditions)
    }
}