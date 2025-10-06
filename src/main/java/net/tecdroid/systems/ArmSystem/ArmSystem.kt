@file:Suppress("MemberVisibilityCanBePrivate")

package net.tecdroid.systems.ArmSystem

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.climber.Climber
import net.tecdroid.subsystems.elevator.Elevator
import net.tecdroid.subsystems.elevatorjoint.ElevatorJoint
import net.tecdroid.subsystems.intake.Intake
import net.tecdroid.subsystems.wrist.Wrist
import net.tecdroid.systems.ArmSystem.ArmMember.*
import net.tecdroid.util.*
import net.tecdroid.util.stateMachine.Phase
import net.tecdroid.util.stateMachine.StateMachine
import net.tecdroid.util.stateMachine.States

enum class ArmMember {
    ArmWrist, ArmElevator, ArmJoint
}

data class ArmPose(
    var wristPosition: Angle,
    var elevatorDisplacement: Distance,
    var elevatorJointPosition: Angle,
    val targetCoralVoltage: Voltage,
    val targetAlgaeVoltage: Voltage,
)
data class ArmOrder(
    val first: ArmMember,
    val second: ArmMember,
    val third: ArmMember
)

enum class ArmPoses(var pose: ArmPose) {
    Passive(
        ArmPose(
            wristPosition           = 0.021.rotations + 5.0.degrees,
            elevatorDisplacement    = 0.01.meters,
            elevatorJointPosition   = 0.25.rotations + 3.5.degrees,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 0.0.volts,
    )
    ),

    FrontL1(
        ArmPose(
            wristPosition           = 55.0.degrees,
            elevatorDisplacement    = 0.75.inches,
            elevatorJointPosition   = 40.0.degrees,
            targetCoralVoltage      = 3.5.volts,
            targetAlgaeVoltage      = 0.0.volts
    )
    ),

    FrontL2(
        ArmPose(
            wristPosition           = 180.0.degrees,
            elevatorDisplacement    = 2.5.inches,
            elevatorJointPosition   = 50.0.degrees,
            targetCoralVoltage      = 5.0.volts,
            targetAlgaeVoltage      = 0.0.volts
        )
    ),

    FrontL3(
        ArmPose(
            wristPosition           = 170.0.degrees,
            elevatorDisplacement    = 14.5.inches,
            elevatorJointPosition   = 62.5.degrees,
            targetCoralVoltage      = 5.0.volts,
            targetAlgaeVoltage      = 0.0.volts
        )
    ),

    FrontL4(
        ArmPose(
            wristPosition           = 140.0.degrees,
            elevatorDisplacement    = 40.0.inches,
            elevatorJointPosition   = 75.0.degrees,
            targetCoralVoltage      = 5.0.volts,
            targetAlgaeVoltage      = 0.0.volts
        )
    ),

    BackL2(
        ArmPose(
            wristPosition           = 220.0.degrees, // TODO("Check the limits of the wrist")
            elevatorDisplacement    = 0.75.inches,
            elevatorJointPosition   = 95.0.degrees, // TODO("Check the limits of the joint")
            targetCoralVoltage      = 5.0.volts,
            targetAlgaeVoltage      = 0.0.volts
    )
    ),

    BackL3(
        ArmPose(
            wristPosition           = 200.0.degrees,
            elevatorDisplacement    = 10.0.inches,
            elevatorJointPosition   = 90.0.degrees,
            targetCoralVoltage      = 5.0.volts,
            targetAlgaeVoltage      = 0.0.volts
    )
    ),

    BackL4(
        ArmPose(
            wristPosition           = 220.0.degrees,
            elevatorDisplacement    = 39.0.inches,
            elevatorJointPosition   = 90.0.degrees,
            targetCoralVoltage      = 5.0.volts,
            targetAlgaeVoltage      = 0.0.volts
    )
    ),

    CoralStation(
        ArmPose(
            wristPosition           = (125 - 67.5).degrees,
            elevatorDisplacement    = 8.0.inches,
            elevatorJointPosition   = 67.5.degrees,
            targetCoralVoltage      = 5.0.volts,
            targetAlgaeVoltage      = 0.0.volts
    )
    ),

    A1(
        ArmPose(
            wristPosition           = 0.2798.rotations,
            elevatorDisplacement    = 0.1457.meters,
            elevatorJointPosition   = 0.1772.rotations - 1.5.degrees,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 5.0.volts
    )
    ),

    A2(
        ArmPose(
            wristPosition           = 0.2628.rotations,
            elevatorDisplacement    = 0.4920.meters,
            elevatorJointPosition   = 0.1968.rotations - 1.5.degrees,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 5.0.volts
    )
    ),

    Processor(
        ArmPose(
            wristPosition           = 0.3705.rotations,
            elevatorDisplacement    = 0.0150.meters,
            elevatorJointPosition   = 0.0415.rotations + 5.0.degrees,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 5.0.volts
    )
    ),

    AlgaeFloorIntake(
        ArmPose(
            wristPosition           = 75.0.degrees,
            elevatorDisplacement    = 1.0.inches,
            elevatorJointPosition   = 10.0.degrees,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 5.0.volts
    )
    ),

    CoralFloorIntake(
        ArmPose(
            wristPosition           = 90.0.degrees,
            elevatorDisplacement    = 0.75.inches,
            elevatorJointPosition   = 0.0.degrees,
            targetCoralVoltage      = 5.0.volts,
            targetAlgaeVoltage      = 0.0.volts
    )
    ),

    Barge(
        ArmPose(
            wristPosition           = 0.3476.rotations,
            elevatorDisplacement    = 1.0420.meters,
            elevatorJointPosition   = 0.263.rotations,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 8.0.volts
    )
    )
}

enum class ArmOrders(val order: ArmOrder) {
    JEW(ArmOrder(ArmJoint, ArmElevator, ArmWrist)),
    JWE(ArmOrder(ArmJoint, ArmWrist, ArmElevator)),
    EWJ(ArmOrder(ArmElevator, ArmWrist, ArmJoint)),
    EJW(ArmOrder(ArmElevator, ArmJoint, ArmWrist)),
    WEJ(ArmOrder(ArmWrist, ArmElevator, ArmJoint)),
    WJE(ArmOrder(ArmWrist, ArmJoint, ArmElevator))
}

enum class PoseCommands(val pose: ArmPose, val order: ArmOrder) {
    L4(ArmPoses.BackL4.pose, ArmOrders.JEW.order),
    L3(ArmPoses.BackL3.pose, ArmOrders.JEW.order),
    L2(ArmPoses.BackL2.pose, ArmOrders.EJW.order),
    CoralStation(ArmPoses.CoralStation.pose, ArmOrders.EJW.order),
    Processor(ArmPoses.Processor.pose, ArmOrders.EJW.order)
}

class ArmSystem(val stateMachine: StateMachine, val limeLightIsAtSetPoint: (Double) -> Boolean, val controller: CompliantXboxController) : Sendable {
    val wrist = Wrist()
    val elevator = Elevator()
    val joint = ElevatorJoint()
    val climber = Climber()
    val intake = Intake()

    private var coralTargetVoltage = 0.0.volts
    private var algaeTargetVoltage = 0.0.volts

    var isScoring = false

    // To change the position orders according to the position of the entire arm
    private var isLow = { false }
    fun setIsLow(value: Boolean) {
        isLow = { value }
    }

    init {
        wrist.matchRelativeEncodersToAbsoluteEncoders()
        joint.matchRelativeEncodersToAbsoluteEncoders()
    }

    fun setJointAngle(angle: Angle): Command = joint.setAngleCommand(angle)
    fun setElevatorDisplacement(displacement: Distance): Command = elevator.setDisplacementCommand(displacement)
    fun setWristAngle(angle: Angle): Command = wrist.setAngleCommand(angle)

    fun enableCoralIntake(): Command = intake.setCoralVoltageCommand(coralTargetVoltage)
    fun enableCoralIntake(voltage: Voltage): Command = intake.setCoralVoltageCommand(voltage)
    fun enableCoralOuttake(): Command = intake.setCoralVoltageCommand(-coralTargetVoltage)

    fun enableAlgaeIntake(): Command = intake.setAlgaeVoltageCommand(algaeTargetVoltage)
    fun enableAlgaeIntake(voltage: Voltage): Command = intake.setAlgaeVoltageCommand(voltage)
    fun enableAlgaeOuttake(): Command = intake.setAlgaeVoltageCommand(-algaeTargetVoltage)

    fun disableCoralIntake() : Command = intake.setCoralVoltageCommand(0.0.volts)
    fun disableAlgaeIntake() : Command = intake.setAlgaeVoltageCommand(0.0.volts)

    private fun getCommandFor(pose: ArmPose, member: ArmMember) : Command = when (member) {
        ArmWrist -> wrist.setAngleCommand(pose.wristPosition).andThen(Commands.waitUntil { wrist.getPositionError() < 50.0.rotations })
        ArmElevator -> elevator.setDisplacementCommand(pose.elevatorDisplacement).andThen(Commands.waitUntil { elevator.getPositionError() < 25.0.rotations })
        ArmJoint -> joint.setAngleCommand(pose.elevatorJointPosition).andThen(Commands.waitUntil { joint.getPositionError() < 25.0.rotations })
    }

    private fun getCommandFor(pose: ArmPose, member: ArmMember, slot: Int) : Command = when (member) {
        ArmWrist -> wrist.setAngleCommand(pose.wristPosition, slot).andThen(Commands.waitUntil { wrist.getPositionError() < 50.0.rotations })
        ArmElevator -> elevator.setDisplacementCommand(pose.elevatorDisplacement).andThen(Commands.waitUntil { elevator.getPositionError() < 25.0.rotations })
        ArmJoint -> joint.setAngleCommand(pose.elevatorJointPosition, slot).andThen(Commands.waitUntil { joint.getPositionError() < 25.0.rotations })
    }

    fun setPoseCommand(pose: ArmPose, order: ArmOrder) : Command {
        return SequentialCommandGroup(
            Commands.runOnce({
                coralTargetVoltage = pose.targetCoralVoltage
                isScoring = when (pose) {
                    ArmPoses.BackL2.pose, ArmPoses.BackL3.pose, ArmPoses.BackL4.pose -> true
                    else -> false
                }
            }),
            getCommandFor(pose, order.first),
            getCommandFor(pose, order.second),
            getCommandFor(pose, order.third))
    }

    fun setPoseCommand(pose: ArmPose, order: ArmOrder, slot: Int) : Command {
        return SequentialCommandGroup(
            Commands.runOnce({
                coralTargetVoltage = pose.targetCoralVoltage
                isScoring = when (pose) {
                    ArmPoses.BackL2.pose, ArmPoses.BackL3.pose, ArmPoses.BackL4.pose -> true
                    else -> false
                }
            }),
            getCommandFor(pose, order.first, slot),
            getCommandFor(pose, order.second, slot),
            getCommandFor(pose, order.third, slot))
    }

    fun setPoseCommand(poseCommand: PoseCommands): Command {
        return setPoseCommand(poseCommand.pose, poseCommand.order)
    }

    fun setPoseAutoCommand(pose: ArmPose, order: ArmOrder) : Command {
        return SequentialCommandGroup(
            Commands.runOnce({ coralTargetVoltage = pose.targetCoralVoltage }),
            getCommandFor(pose, order.first),
            getCommandFor(pose, order.second),
            getCommandFor(pose, order.third),
        )
    }

    fun editJointPose(pose: ArmPose, angleDelta: Angle) : ArmPose {
        return pose.copy(elevatorJointPosition = pose.elevatorJointPosition.plus(angleDelta))
    }

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Elevator Error (Rotations)", { elevator.getPositionError().`in`(Rotations) }) {}
            addDoubleProperty("Joint Error (Rotations)", { joint.getPositionError().`in`(Rotations) }) {}
            addDoubleProperty("Wrist Error (Rotations)", { wrist.getPositionError().`in`(Rotations) }) {}
            addDoubleProperty("Elevator Displacement (Meters)", { elevator.displacement.`in`(Meters) }) {}
            addDoubleProperty("Joint Position (Rotations)", { joint.angle.`in`(Rotations) }) {}
            addDoubleProperty("Wrist Position (Rotations)", { wrist.angle.`in`(Rotations) }) {}
        }
    }

    fun publishShuffleBoardData() {
        val tab = Shuffleboard.getTab("Driver Tab")
        tab.addBoolean("coral", { getSensorRead() })
        tab.addBoolean("llIsAtSetPoint", { limeLightIsAtSetPoint(0.1)})
        tab.addString("State", { stateMachine.getCurrentState().toString() })
        tab.addDouble("Target Voltage") { coralTargetVoltage.`in`(Volts) }
    }

    fun getSensorRead() : Boolean = intake.hasCoral()

    fun scoringSequence(pose: PoseCommands): Command {
        return SequentialCommandGroup(
            setPoseCommand(pose).andThen(WaitCommand(0.15.seconds)).andThen(enableCoralIntake()),
            WaitUntilCommand { intake.hasCoral().not() }.andThen(WaitCommand(0.05.seconds)).andThen(disableCoralIntake()),
            setPoseCommand(PoseCommands.CoralStation)
        )
    }

    /*fun scoringSequence(pose: PoseCommands): Command {
        return setPoseCommand(pose).andThen(WaitCommand(0.15.seconds)).andThen(enableIntake())
    }*/

    fun scoringSequence(pose: () -> PoseCommands): Command {
        return SequentialCommandGroup(
            setPoseCommand(pose.invoke()).andThen(WaitCommand(0.15.seconds)).andThen(enableCoralIntake()),
            WaitUntilCommand { intake.hasCoral().not() }.andThen(WaitCommand(0.05.seconds)).andThen(disableCoralIntake()),
            setPoseCommand(PoseCommands.CoralStation)
        )
    }

    fun changeState() {
        when(stateMachine.getCurrentState()) {
            States.AlgaeState -> stateMachine.changeState(States.CoralState)
            else -> stateMachine.changeState(States.AlgaeState)
        }
    }

    // Used to avoid the one command binding of the trigger, and process the logic out of the trigger command
    private fun scheduleCMD(command: Command) = command.schedule()

    private fun assignStatesCommands() {
        // Active passive intake
        States.AlgaeState.setInitialCommand(intake.setAlgaeVoltageCommand(1.5.volts))

        // Go to passive position after score a coral
        States.ScoreState.setEndCommand(SequentialCommandGroup(
            WaitCommand(0.05.seconds),
            disableCoralIntake(),
            setPoseCommand(PoseCommands.CoralStation)))
        // Set a physical condition for triggering the climb state
        stateMachine.addCondition({ controller.rightTrigger().asBoolean && controller.leftTrigger().asBoolean }, States.ClimbState,
            Phase.Teleop )
        // When climb state is triggered, the processor pose will be scheduled so climber is able to work.
        States.ClimbState.setInitialCommand(setPoseCommand(PoseCommands.Processor))

        // Change to score state when coral is detected
        stateMachine.addCondition({ getSensorRead() }, States.ScoreState, Phase.Teleop)

        // Change to coral state if we are in score state, and we just pull out a coral
        stateMachine.addCondition({ stateMachine.isState(States.ScoreState).invoke() && !getSensorRead() }, States.CoralState, Phase.Teleop)

    }

    fun setAllCoast(): Command {
        return SequentialCommandGroup(
            wrist.coast(),
            elevator.coast(),
            joint.coast(),
            climber.coast(),
        ).ignoringDisable(true)
    }

    fun setAllBrake(): Command {
        return SequentialCommandGroup(
            wrist.brake(),
            elevator.brake(),
            joint.brake(),
            climber.brake(),
        ).ignoringDisable(true)
    }

    fun assignCommands() {
        assignStatesCommands()

        controller.povLeft().onTrue(
            Commands.runOnce({changeState()})
        )

        // Y
        controller.y().onTrue(
            Commands.runOnce({
                scheduleCMD(when(stateMachine.getCurrentState()){
                    States.ScoreState -> Commands.either(
                        scoringSequence(PoseCommands.L4),
                        setPoseCommand(PoseCommands.L4),
                        { limeLightIsAtSetPoint(0.75) })

                    States.CoralState -> setPoseCommand(PoseCommands.L4)
                    States.IntakeState -> setPoseCommand(PoseCommands.L4)
                        .andThen({stateMachine.changeState(States.CoralState)})
                    States.AlgaeState -> Commands.sequence(
                        enableAlgaeIntake(2.0.volts),
                        setPoseCommand(
                            ArmPoses.Barge.pose,
                            ArmOrders.JEW.order
                        ).andThen({ setIsLow(false) }),
                        disableAlgaeIntake())

                    States.ClimbState -> Commands.none()
                })
            })
        )

        // B
        controller.b().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.ScoreState -> Commands.either(
                    scoringSequence(PoseCommands.L3),
                    setPoseCommand(PoseCommands.L3),
                    { limeLightIsAtSetPoint(0.75) })

                States.CoralState -> setPoseCommand(PoseCommands.L3)
                States.IntakeState -> setPoseCommand(PoseCommands.L3)
                    .andThen({stateMachine.changeState(States.CoralState)})
                States.AlgaeState -> Commands.sequence(
                    enableAlgaeIntake(2.0.volts),
                    setPoseCommand(
                        ArmPoses.A2.pose,
                        if (isLow()) ArmOrders.JWE.order else ArmOrders.EWJ.order
                    ).andThen({ setIsLow(false) }),
                    disableAlgaeIntake())

                States.ClimbState -> Commands.none()
            })
        }))

        // A
        controller.a().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.ScoreState -> Commands.either(
                    scoringSequence(PoseCommands.L2),
                    setPoseCommand(PoseCommands.L2),
                    { limeLightIsAtSetPoint(0.75) })

                States.CoralState -> setPoseCommand(PoseCommands.L2)
                States.IntakeState -> setPoseCommand(PoseCommands.L2)
                    .andThen({stateMachine.changeState(States.CoralState)})
                States.AlgaeState -> Commands.sequence(
                    enableAlgaeIntake(2.0.volts),
                    setPoseCommand(
                        ArmPoses.A1.pose,
                        if (isLow()) ArmOrders.JWE.order else ArmOrders.EWJ.order
                    ).andThen({ setIsLow(false) }),
                    disableAlgaeIntake())

                States.ClimbState -> Commands.none()
            })
        }))

        // X
        controller.x().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.CoralState -> setPoseCommand(PoseCommands.CoralStation)
                    .andThen({ setIsLow(true) })
                    .andThen(Commands.runOnce({ stateMachine.changeState(States.IntakeState)}))

                States.IntakeState, States.ScoreState -> setPoseCommand(PoseCommands.CoralStation)
                    .andThen({ setIsLow(true) })

                States.AlgaeState -> Commands.sequence(
                    setPoseCommand(
                        ArmPoses.CoralStation.pose,
                        ArmOrders.EJW.order
                    ),
                    Commands.runOnce({ stateMachine.changeState(States.IntakeState)}))

                States.ClimbState -> InstantCommand({ climber.setClimberWristAngle(45.0.degrees) })
            })
        }))

        // Intake
        controller.rightBumper().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.CoralState -> Commands.parallel(
                    setPoseCommand(PoseCommands.CoralStation)
                        .andThen({ setIsLow(true) }),
                    Commands.runOnce({ stateMachine.changeState(States.IntakeState)}),
                    enableCoralIntake()
                )
                States.ClimbState -> InstantCommand({climber.setClimberRollersVoltage(8.0.volts)})
                else -> enableCoralIntake()
            })
        })).onFalse(disableCoralIntake())

        // POV down
        controller.povDown().onTrue(
            Commands.sequence(
                enableAlgaeIntake(3.0.volts),
                setPoseCommand(ArmPoses.AlgaeFloorIntake.pose, ArmOrders.EWJ.order, 1),
                disableAlgaeIntake(),
                Commands.runOnce({ stateMachine.changeState(States.AlgaeState) })
            )
        )

        // POV right
        controller.povRight().onTrue(
            Commands.sequence(
                enableAlgaeIntake(3.0.volts),
                setPoseCommand(ArmPoses.Processor.pose, ArmOrders.EWJ.order, 1),
                disableAlgaeIntake(),
                Commands.runOnce({ stateMachine.changeState(States.AlgaeState) })
            )
        )

        // POV up
        controller.povUp().onTrue(
            Commands.sequence(
                setPoseCommand(ArmPoses.CoralFloorIntake.pose, ArmOrders.EWJ.order)
            )
        )

        controller.leftBumper().onTrue(enableCoralOuttake()).onFalse(disableCoralIntake())
    }

}