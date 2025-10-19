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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
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
            wristPosition           = 110.0.degrees,
            elevatorDisplacement    = 0.0.inches,
            elevatorJointPosition   = 60.0.degrees,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 0.0.volts,
    )
    ),

    BackL2(
        ArmPose(
            wristPosition           = 130.0.degrees,
            elevatorDisplacement    = 0.0.inches,
            elevatorJointPosition   = 90.0.degrees,
            targetCoralVoltage      = 8.0.volts,
            targetAlgaeVoltage      = 8.0.volts
    )
    ),

    BackL2Safe(
        ArmPose(
            wristPosition           = 100.0.degrees,
            elevatorDisplacement    = 0.0.inches,
            elevatorJointPosition   = 90.0.degrees,
            targetCoralVoltage      = 8.0.volts,
            targetAlgaeVoltage      = 8.0.volts
        )
    ),

    BackL3(
        ArmPose(
            wristPosition           = 110.0.degrees,
            elevatorDisplacement    = 13.25.inches + 0.03.meters,
            elevatorJointPosition   = 90.0.degrees,
            targetCoralVoltage      = 8.0.volts,
            targetAlgaeVoltage      = 8.0.volts
    )
    ),

    BackL4(
        ArmPose(
            wristPosition           = 130.0.degrees,
            elevatorDisplacement    = 40.3.inches,
            elevatorJointPosition   = 91.0.degrees,
            targetCoralVoltage      = 8.0.volts,
            targetAlgaeVoltage      = 8.0.volts
    )
    ),

    CoralStation(
        ArmPose(
            wristPosition           = (-22.5).degrees,
            elevatorDisplacement    = 0.17.meters,
            elevatorJointPosition   = 70.0.degrees,
            targetCoralVoltage      = 3.75.volts,
            targetAlgaeVoltage      = 3.75.volts
    )
    ),

    A1(
        ArmPose(
            wristPosition           = (-54.0).degrees,
            elevatorDisplacement    = 0.195.meters,
            elevatorJointPosition   = 65.834.degrees,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 8.0.volts
    )
    ),

    A2(
        ArmPose(
            wristPosition           = (-32.218).degrees,
            elevatorDisplacement    = 0.42.meters,
            elevatorJointPosition   = 69.262.degrees,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 8.0.volts
    )
    ),

    Processor(
        ArmPose(
            wristPosition           = 0.3705.rotations,
            elevatorDisplacement    = 0.0.meters,
            elevatorJointPosition   = 0.4437.degrees,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 4.0.volts
    )
    ),

    AlgaeFloorIntake(
        ArmPose(
            wristPosition           = (-35.087).degrees,
            elevatorDisplacement    = 0.045.meters,
            elevatorJointPosition   = 14.24.degrees,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 4.0.volts
    )
    ),

    CoralFloorIntake(
        ArmPose(
            wristPosition           = 0.0.degrees,
            elevatorDisplacement    = 0.0.inches,
            elevatorJointPosition   = 0.1.degrees,
            targetCoralVoltage      = 6.0.volts,
            targetAlgaeVoltage      = 12.0.volts
    )
    ),

    CoralFloorIntakeSafe(
        ArmPose(
            wristPosition           = 110.0.degrees,
            elevatorDisplacement    = 0.0.inches,
            elevatorJointPosition   = 0.5.degrees,
            targetCoralVoltage      = 6.0.volts,
            targetAlgaeVoltage      = 12.0.volts
    )
    ),


    Barge(
        ArmPose(
            wristPosition           = 5.95.degrees,
            elevatorDisplacement    = 1.0293.meters,
            elevatorJointPosition   = 90.0.degrees,
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

enum class PoseCommands(val pose: ArmPoses, val order: ArmOrder) {
    BackL4(ArmPoses.BackL4, ArmOrders.JEW.order),
    BackL3(ArmPoses.BackL3, ArmOrders.JEW.order),
    BackL2(ArmPoses.BackL2, ArmOrders.JEW.order),
    A1(ArmPoses.A1, ArmOrders.JEW.order),
    A2(ArmPoses.A2, ArmOrders.JEW.order),
    AlgaeFloorIntake(ArmPoses.AlgaeFloorIntake, ArmOrders.EJW.order),
    Barge(ArmPoses.Barge, ArmOrders.JWE.order),
    CoralFloorIntake(ArmPoses.CoralFloorIntake, ArmOrders.EWJ.order),
    CoralFloorIntakeSafe(ArmPoses.CoralFloorIntakeSafe, ArmOrders.EWJ.order),
    CoralStation(ArmPoses.CoralStation, ArmOrders.EJW.order),
    Processor(ArmPoses.Processor, ArmOrders.EJW.order),
    Passive(ArmPoses.Passive, ArmOrders.EJW.order),
}

class ArmSystem(val stateMachine: StateMachine, val limeLightIsAtSetPoint: (Distance) -> Boolean, val controller: CompliantXboxController) : Sendable {
    val wrist = Wrist(stateMachine.isState(States.ClimbState))
    val elevator = Elevator()
    val joint = ElevatorJoint()
    val climber = Climber()
    val intake = Intake(stateMachine.isState(States.ClimbState))

    private var coralTargetVoltage = 0.0.volts
    private var algaeTargetVoltage = 0.0.volts

    var isScoring = false
    val climbTrigger = Trigger{ controller.leftStick().asBoolean && controller.rightStick().asBoolean && hasCoral().not() }

    var currentPose = ArmPoses.Passive
    var targetPose = ArmPoses.BackL2

    // To change the position orders according to the position of the entire arm
    private var isLow = { false }
    fun setIsLow(value: Boolean) {
        isLow = { value }
    }

    init {
        wrist.matchRelativeEncodersToAbsoluteEncoders()
        joint.matchRelativeEncodersToAbsoluteEncoders()
        climber.matchRelativeEncodersToAbsoluteEncoders()
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

    fun setPoseCommand(pose: ArmPoses, order: ArmOrder) : Command {
        return when(currentPose) {
            // When is in BackL2 to prevent the wires from colliding
            ArmPoses.BackL2, ArmPoses.BackL3, ArmPoses.BackL4 -> SequentialCommandGroup(
                getCommandFor(ArmPoses.BackL2Safe.pose, ArmOrders.WEJ.order.first),
                Commands.runOnce({
                    currentPose = pose
                    coralTargetVoltage = pose.pose.targetCoralVoltage
                    algaeTargetVoltage = pose.pose.targetAlgaeVoltage
                    isScoring = when (pose) {
                        ArmPoses.BackL2, ArmPoses.BackL3, ArmPoses.BackL4 -> true
                        else -> false
                    }
                }),
                getCommandFor(pose.pose, order.first),
                getCommandFor(pose.pose, order.second),
                getCommandFor(pose.pose, order.third))
            // rest of cases
            ArmPoses.Barge -> SequentialCommandGroup(
                Commands.runOnce({
                    currentPose = pose
                    coralTargetVoltage = pose.pose.targetCoralVoltage
                    algaeTargetVoltage = pose.pose.targetAlgaeVoltage
                    isScoring = when (pose) {
                        ArmPoses.BackL2, ArmPoses.BackL3, ArmPoses.BackL4 -> true
                        else -> false
                    }
                }),
                getCommandFor(pose.pose, ArmElevator),
                getCommandFor(pose.pose, ArmWrist),
                getCommandFor(pose.pose, ArmJoint))
            else -> SequentialCommandGroup(
                Commands.runOnce({
                    currentPose = pose
                    coralTargetVoltage = pose.pose.targetCoralVoltage
                    algaeTargetVoltage = pose.pose.targetAlgaeVoltage
                    isScoring = when (pose) {
                        ArmPoses.BackL2, ArmPoses.BackL3, ArmPoses.BackL4 -> true
                        else -> false
                    }
                }),
                getCommandFor(pose.pose, order.first),
                getCommandFor(pose.pose, order.second),
                getCommandFor(pose.pose, order.third))
        }
    }

    fun setPoseCommand(pose: ArmPoses, order: ArmOrder, slot: Int) : Command {
        return SequentialCommandGroup(
            Commands.runOnce({
                currentPose = pose
                coralTargetVoltage = pose.pose.targetCoralVoltage
                algaeTargetVoltage = pose.pose.targetAlgaeVoltage
                isScoring = when (pose) {
                    ArmPoses.BackL2, ArmPoses.BackL3, ArmPoses.BackL4 -> true
                    else -> false
                }
            }),
            getCommandFor(pose.pose, order.first, slot),
            getCommandFor(pose.pose, order.second, slot),
            getCommandFor(pose.pose, order.third, slot))
    }

    fun setPoseCommand(poseCommand: PoseCommands): Command {
        return setPoseCommand(poseCommand.pose, poseCommand.order)
    }

    fun setPoseAutoCommand(pose: ArmPoses, order: ArmOrder) : Command {
        return SequentialCommandGroup(
            Commands.runOnce({ currentPose = pose }),
            Commands.runOnce({ coralTargetVoltage = pose.pose.targetCoralVoltage }),
            Commands.runOnce({ algaeTargetVoltage = pose.pose.targetAlgaeVoltage }),
            getCommandFor(pose.pose, order.first),
            getCommandFor(pose.pose, order.second),
            getCommandFor(pose.pose, order.third),
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
        tab.addBoolean("coral", { hasCoral() })
        tab.addBoolean("llIsAtSetPoint", { limeLightIsAtSetPoint(0.2.meters)})
        tab.addString("State", { stateMachine.getCurrentState().toString() })
        tab.addDouble("Target Coral Voltage") { coralTargetVoltage.`in`(Volts) }
        tab.addDouble("Target Algae Voltage") { algaeTargetVoltage.`in`(Volts) }
        tab.addDouble("Target Wrist Pos") { currentPose.pose.wristPosition.`in`(Degrees) }
        tab.addString("Current arm pose") { currentPose.toString() }
        tab.addString("Target arm pose") { targetPose.toString() }
    }

    fun hasCoral() : Boolean = intake.hasCoral()

    fun scoringSequence(pose: PoseCommands): Command {
        return setPoseCommand(pose).andThen(WaitCommand(0.5.seconds)).andThen(Commands.runOnce({
            scheduleCMD(ParallelCommandGroup(enableCoralOuttake(), enableAlgaeOuttake()))
        }))
    }

    fun scoringSequence(pose: ArmPoses, order: ArmOrder): Command {
        return setPoseCommand(pose, order).andThen(WaitCommand(0.5.seconds)).andThen(Commands.runOnce({
            scheduleCMD(ParallelCommandGroup(enableCoralOuttake(), enableAlgaeOuttake()))
        }))
    }

    fun scoringSequence(pose: () -> PoseCommands): Command {
        return setPoseCommand(pose.invoke()).andThen(WaitCommand(0.5.seconds)).andThen(enableCoralOuttake())
    }

    // Used to avoid the one command binding of the trigger, and process the logic out of the trigger command
    private fun scheduleCMD(command: Command) = command.schedule()

    // ! State machine

    private fun assignStatesCommands() {
        // Active passive intake

        States.ScoreState.setInitialCommand(setPoseCommand(PoseCommands.Passive))

        // Go to passive position after score a coral
        States.ScoreState.setEndCommand(SequentialCommandGroup(
            WaitUntilCommand({ hasCoral().not() }),
            WaitCommand(0.1.seconds),
            intake.setAlgaeVoltageCommand(0.0.volts),
            intake.setCoralVoltageCommand(0.0.volts),
            setPoseCommand(ArmPoses.CoralFloorIntakeSafe, ArmOrders.EWJ.order)
        ))

        // Set a physical condition for triggering the climb state
        stateMachine.addCondition({ climbTrigger.asBoolean }, States.ClimbState, Phase.Teleop )

        // Change to score state when coral is detected
        stateMachine.addCondition({ hasCoral() }, States.ScoreState, Phase.Teleop)

        // Change to coral state if we are in score state, and we just pull out a coral
        stateMachine.addCondition({ stateMachine.isState(States.ScoreState).invoke() && !hasCoral() },
            States.MarcoState, Phase.Teleop)
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
        // Y
        controller.y().onTrue(
            Commands.runOnce({
                scheduleCMD(when(stateMachine.getCurrentState()){
                    States.ScoreState -> Commands.either (
                        scoringSequence(PoseCommands.BackL4),
                        Commands.runOnce({ targetPose = ArmPoses.BackL4 }),
                        { limeLightIsAtSetPoint(0.415.meters) }
                    )

                    States.MarcoState -> setPoseCommand(PoseCommands.BackL4)
                    States.IntakeState -> setPoseCommand(PoseCommands.BackL4)
                        .andThen({stateMachine.changeState(States.MarcoState)})

                    // angle to climb is 33.5 deg
                    // made to go 6deg inside
                    States.ClimbState -> Commands.none()
                })
            })
        )

        // B
        controller.b().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.ScoreState -> Commands.either (
                    scoringSequence(PoseCommands.BackL3),
                    Commands.runOnce({ targetPose = ArmPoses.BackL3 }),
                    { limeLightIsAtSetPoint(0.445.meters) }
                )

                States.MarcoState -> setPoseCommand(PoseCommands.BackL3)
                States.IntakeState -> setPoseCommand(PoseCommands.BackL3)
                    .andThen({stateMachine.changeState(States.MarcoState)})
                States.ClimbState -> Commands.none()
            })
        }))

        // A
        controller.a().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.ScoreState -> Commands.either (
                    scoringSequence(PoseCommands.BackL2),
                    Commands.runOnce({ targetPose = ArmPoses.BackL2 }),
                    { limeLightIsAtSetPoint((-0.145).meters) }
                )

                States.MarcoState -> setPoseCommand(ArmPoses.BackL2Safe, ArmOrders.EJW.order)
                    .andThen(setPoseCommand(PoseCommands.BackL2))

                States.IntakeState -> setPoseCommand(ArmPoses.BackL2Safe, ArmOrders.EJW.order)
                    .andThen(setPoseCommand(PoseCommands.BackL2))
                    .andThen({stateMachine.changeState(States.MarcoState)})
                States.ClimbState -> Commands.none()
            })
        }))

        // X
        controller.x().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.MarcoState -> SequentialCommandGroup(

                    //TODO: CJ
                    Commands.runOnce({ stateMachine.changeState(States.IntakeState)}),
                    setPoseCommand(ArmPoses.CoralFloorIntakeSafe, ArmOrders.EWJ.order)
                )

                States.IntakeState -> setPoseCommand(ArmPoses.CoralFloorIntakeSafe, ArmOrders.EWJ.order)

                States.ScoreState -> Commands.none()

                States.ClimbState -> setPoseCommand(ArmPoses.CoralFloorIntakeSafe, ArmOrders.EWJ.order)
            })
        }))

        controller.rightBumper()
            .onTrue(Commands.runOnce({scheduleCMD(when (stateMachine.getCurrentState()) {

                            States.ScoreState -> Commands.runOnce({scheduleCMD(
                                ParallelCommandGroup(enableCoralOuttake(), enableAlgaeOuttake()))
                            })

                            States.ClimbState -> Commands.runOnce({scheduleCMD(
                                InstantCommand({climber.setClimberRollersVoltage(12.0.volts)})
                            )})

                            else -> ParallelCommandGroup(
                                setPoseCommand(ArmPoses.CoralFloorIntake, ArmOrders.EJW.order),
                                Commands.runOnce({scheduleCMD(
                                    ParallelCommandGroup(enableCoralIntake(), enableAlgaeIntake()))
                            }))
                        }
                    )
                })
            )
            .onFalse(Commands.runOnce({
                scheduleCMD(
                    when (stateMachine.getCurrentState()) {
                        States.ScoreState -> Commands.runOnce({scheduleCMD(
                                ParallelCommandGroup(disableCoralIntake(), disableAlgaeIntake()))
                        })

                        States.ClimbState -> Commands.runOnce({scheduleCMD(
                            InstantCommand({climber.setClimberRollersVoltage(0.0.volts)})
                        )})

                        else -> ParallelCommandGroup(
                            Commands.runOnce({
                                scheduleCMD(ParallelCommandGroup(disableCoralIntake(), disableAlgaeIntake()))
                            }),
                            setPoseCommand(ArmPoses.CoralFloorIntakeSafe, ArmOrders.WEJ.order)
                        )
                    }
                )
            }))

            // ! Analog climber
        controller.povUp().whileTrue(Commands.run({
            scheduleCMD(
                when (stateMachine.getCurrentState()) {
                    States.ClimbState -> InstantCommand({ climber.setVoltage((-12.0).volts) })
                    else -> Commands.none()
                }
            )}))
                .onFalse(InstantCommand({ climber.setVoltage(0.0.volts) }))


        controller.povDown().whileTrue(Commands.run({
            scheduleCMD(
                when (stateMachine.getCurrentState()) {
                    States.ClimbState -> InstantCommand({ climber.setVoltage(12.0.volts) })
                    else -> enableAlgaeOuttake()
                }
            )}))
            .onFalse(InstantCommand({ climber.setVoltage(0.0.volts) }))

        controller.leftBumper()
            .onTrue(Commands.runOnce({scheduleCMD(when (stateMachine.getCurrentState()) {
                States.ScoreState -> scoringSequence(targetPose, ArmOrders.JEW.order)
                else -> Commands.none()
            })}))

        controller.povRight().onTrue(SequentialCommandGroup(
            setPoseCommand(ArmPoses.A1, ArmOrders.JEW.order),
            enableAlgaeIntake((-6.0).volts)
        )
        )
        controller.povLeft().onTrue(Commands.runOnce({ wrist.onMatchRelativeEncodersToAbsoluteEncoders() }) )
    }
}