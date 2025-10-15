@file:Suppress("MemberVisibilityCanBePrivate")

package net.tecdroid.systems.ArmSystem

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ScheduleCommand
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

    FrontL1(
        ArmPose(
            wristPosition           = 55.0.degrees - 90.0.degrees,
            elevatorDisplacement    = 0.75.inches,
            elevatorJointPosition   = 40.0.degrees,
            targetCoralVoltage      = 6.0.volts,
            targetAlgaeVoltage      = 6.0.volts
    )
    ),

    FrontL2(
        ArmPose(
            wristPosition           = 180.0.degrees - 90.0.degrees,
            elevatorDisplacement    = 2.5.inches,
            elevatorJointPosition   = 50.0.degrees,
            targetCoralVoltage      = 8.0.volts,
            targetAlgaeVoltage      = 8.0.volts
        )
    ),

    FrontL3(
        ArmPose(
            wristPosition           = 170.0.degrees - 90.0.degrees,
            elevatorDisplacement    = 14.5.inches,
            elevatorJointPosition   = 62.5.degrees,
            targetCoralVoltage      = 8.0.volts,
            targetAlgaeVoltage      = 8.0.volts
        )
    ),

    FrontL4(
        ArmPose(
            wristPosition           = 140.0.degrees - 90.0.degrees,
            elevatorDisplacement    = 40.0.inches,
            elevatorJointPosition   = 75.0.degrees,
            targetCoralVoltage      = 8.0.volts,
            targetAlgaeVoltage      = 8.0.volts
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
            wristPosition           = 200.0.degrees - 90.0.degrees,
            elevatorDisplacement    = 14.5.inches,
            elevatorJointPosition   = 90.0.degrees,
            targetCoralVoltage      = 8.0.volts,
            targetAlgaeVoltage      = 8.0.volts
    )
    ),

    BackL4(
        ArmPose(
            wristPosition           = 220.0.degrees - 90.0.degrees,
            elevatorDisplacement    = 40.3.inches,
            elevatorJointPosition   = 90.0.degrees,
            targetCoralVoltage      = 8.0.volts,
            targetAlgaeVoltage      = 8.0.volts
    )
    ),

    CoralStation(
        ArmPose(
            wristPosition           = (125 - 67.5).degrees - 90.0.degrees,
            elevatorDisplacement    = 8.0.inches,
            elevatorJointPosition   = 67.5.degrees,
            targetCoralVoltage      = 8.0.volts,
            targetAlgaeVoltage      = 8.0.volts
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
            wristPosition           = 75.0.degrees - 90.0.degrees,
            elevatorDisplacement    = 1.0.inches,
            elevatorJointPosition   = 10.0.degrees,
            targetCoralVoltage      = 0.0.volts,
            targetAlgaeVoltage      = 5.0.volts
    )
    ),

    CoralFloorIntake(
        ArmPose(
            wristPosition           = -(1.0).degrees,
            elevatorDisplacement    = 0.0.inches,
            elevatorJointPosition   = 0.5.degrees,
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

enum class PoseCommands(val pose: ArmPoses, val order: ArmOrder) {
    // Todo Need to check pose order
    FrontL4(ArmPoses.FrontL4, ArmOrders.JEW.order),
    FrontL3(ArmPoses.FrontL3, ArmOrders.JEW.order),
    FrontL2(ArmPoses.FrontL2, ArmOrders.JEW.order),
    //
    BackL4(ArmPoses.BackL4, ArmOrders.JEW.order),
    BackL3(ArmPoses.BackL3, ArmOrders.JEW.order),
    BackL2(ArmPoses.BackL2, ArmOrders.JEW.order),
    CoralFloorIntakeSafe(ArmPoses.CoralFloorIntakeSafe, ArmOrders.EWJ.order),
    CoralStation(ArmPoses.CoralStation, ArmOrders.EJW.order),
    Processor(ArmPoses.Processor, ArmOrders.EJW.order),
    Passive(ArmPoses.Passive, ArmOrders.JEW.order),
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
    }

    fun hasCoral() : Boolean = intake.hasCoral()

    fun scoringSequence(pose: PoseCommands): Command {
        return setPoseCommand(pose).andThen(WaitCommand(0.2.seconds)).andThen(Commands.runOnce({
            scheduleCMD(ParallelCommandGroup(enableCoralOuttake(), enableAlgaeOuttake()))
        }))
    }

    fun scoringSequence(pose: () -> PoseCommands): Command {
        return setPoseCommand(pose.invoke()).andThen(WaitCommand(0.15.seconds)).andThen(enableCoralOuttake())
    }

    fun changeState() {
        when(stateMachine.getCurrentState()) {
            States.AlgaeState -> stateMachine.changeState(States.CoralState)
            else -> stateMachine.changeState(States.AlgaeState)
        }
    }

    // Used to avoid the one command binding of the trigger, and process the logic out of the trigger command
    private fun scheduleCMD(command: Command) = command.schedule()

    // ! State machine

    private fun assignStatesCommands() {
        // Active passive intake
        States.AlgaeState.setInitialCommand(intake.setAlgaeVoltageCommand(1.5.volts))

        States.ScoreState.setInitialCommand(InstantCommand({ CommandScheduler.getInstance().clearComposedCommands()})
            .andThen(setPoseCommand(PoseCommands.Passive)))

        // Go to passive position after score a coral
        States.ScoreState.setEndCommand(SequentialCommandGroup(
            WaitUntilCommand({ hasCoral().not() }),
            WaitCommand(0.1.seconds),
            intake.setAlgaeVoltageCommand(0.0.volts),
            intake.setCoralVoltageCommand(0.0.volts),
            setPoseCommand(ArmPoses.CoralFloorIntakeSafe, ArmOrders.EJW.order)
        ))


        // Set a physical condition for triggering the climb state
        stateMachine.addCondition({ climbTrigger.asBoolean }, States.ClimbState, Phase.Teleop )
        // When climb state is triggered, the coral safe intake pose will be scheduled so climber is able to work.
        /*States.ClimbState.setInitialCommand(SequentialCommandGroup(
            setPoseCommand(PoseCommands.CoralFloorIntakeSafe),
            Commands.run({
                scheduleCMD(
                    Commands.run({ climber.setRawAngle(140.0.degrees, 8.0.volts) }))
            }),
            Commands.
            run({
                scheduleCMD(
                    Commands.run({ climber.setClimberRollersVoltage(12.0.volts) }))
            })
        ))*/

        // Change to score state when coral is detected
        stateMachine.addCondition({ hasCoral() }, States.ScoreState, Phase.Teleop)

        // Change to coral state if we are in score state, and we just pull out a coral
        stateMachine.addCondition({ stateMachine.isState(States.ScoreState).invoke() && !hasCoral() }, States.CoralState, Phase.Teleop)

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
                    States.ScoreState -> Commands.either (
                        scoringSequence(PoseCommands.BackL4),
                        setPoseCommand(PoseCommands.BackL4),
                        { limeLightIsAtSetPoint(0.415.meters) }
                    )
                        //WaitUntilCommand { limeLightIsAtSetPoint(0.1.meters) }.andThen(
                        //scoringSequence(PoseCommands.BackL4))

                    States.CoralState -> setPoseCommand(PoseCommands.BackL4)
                    States.IntakeState -> setPoseCommand(PoseCommands.BackL4)
                        .andThen({stateMachine.changeState(States.CoralState)})
                    States.AlgaeState -> Commands.none()//Commands.sequence(
//                        enableAlgaeIntake(2.0.volts),
//                        setPoseCommand(
//                            ArmPoses.Barge.pose,
//                            ArmOrders.JEW.order
//                        ).andThen({ setIsLow(false) }),
//                        disableAlgaeIntake())

                    // angle to climb is 33.5 deg
                    // made to go 6deg inside
                    States.ClimbState -> Commands.run({ climber.setAngle(40.0.degrees) })
                })
            })
        )

        // B
        controller.b().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.ScoreState -> Commands.either (
                    scoringSequence(PoseCommands.BackL3),
                    setPoseCommand(PoseCommands.BackL3),
                    { limeLightIsAtSetPoint(0.445.meters) }
                )

                    //WaitUntilCommand { limeLightIsAtSetPoint(0.13.meters) }.andThen(
                        //scoringSequence(PoseCommands.BackL3))

                States.CoralState -> setPoseCommand(PoseCommands.BackL3)
                States.IntakeState -> setPoseCommand(PoseCommands.BackL3)
                    .andThen({stateMachine.changeState(States.CoralState)})
                States.AlgaeState -> Commands.none()//Commands.sequence(
//                    enableAlgaeIntake(2.0.volts),
//                    setPoseCommand(
//                        ArmPoses.A2.pose,
//                        if (isLow()) ArmOrders.JWE.order else ArmOrders.EWJ.order
//                    ).andThen({ setIsLow(false) }),
//                    disableAlgaeIntake())

                States.ClimbState -> Commands.none()
            })
        }))

        // A
        controller.a().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.ScoreState -> Commands.either (
                    scoringSequence(PoseCommands.BackL2),
                    setPoseCommand(PoseCommands.BackL2),
                    { limeLightIsAtSetPoint((-0.145).meters) }
                )
                    //WaitUntilCommand { limeLightIsAtSetPoint(0.25.meters) }.andThen(
                    //scoringSequence(PoseCommands.BackL2))

                States.CoralState -> setPoseCommand(ArmPoses.BackL2Safe, ArmOrders.EJW.order)
                    .andThen(setPoseCommand(PoseCommands.BackL2))
                States.IntakeState -> setPoseCommand(ArmPoses.BackL2Safe, ArmOrders.EJW.order)
                    .andThen(setPoseCommand(PoseCommands.BackL2))
                    .andThen({stateMachine.changeState(States.CoralState)})
                States.AlgaeState -> Commands.none()//Commands.sequence(
//                    enableAlgaeIntake(2.0.volts),
//                    setPoseCommand(
//                        ArmPoses.A1.pose,
//                        if (isLow()) ArmOrders.JWE.order else ArmOrders.EWJ.order
//                    ).andThen({ setIsLow(false) }),
//                    disableAlgaeIntake())

                States.ClimbState -> Commands.none()
            })
        }))

        // X
        controller.x().onTrue(Commands.runOnce({
            scheduleCMD(when(stateMachine.getCurrentState()){
                States.CoralState -> setPoseCommand(ArmPoses.CoralFloorIntakeSafe, ArmOrders.EJW.order)
                    .andThen({ setIsLow(true) })
                    .andThen(Commands.runOnce({ stateMachine.changeState(States.IntakeState)}))

                States.IntakeState, States.ScoreState -> setPoseCommand(ArmPoses.CoralFloorIntakeSafe, ArmOrders.EJW.order)
                    .andThen({ setIsLow(true) })

                States.AlgaeState -> Commands.sequence(
                    setPoseCommand(ArmPoses.CoralFloorIntakeSafe, ArmOrders.EJW.order),
                    Commands.runOnce({ stateMachine.changeState(States.IntakeState)}))

                States.ClimbState -> Commands.run({ climber.setAngle(80.0.degrees) })
            })
        }))

        // Intake
        /*controller.rightBumper().onTrue(Commands.runOnce({
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
        })).onFalse(disableCoral
        Intake())*/
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
        controller.povUp().onTrue(Commands.runOnce({
            scheduleCMD(
                when (stateMachine.getCurrentState()) {
                    States.ClimbState -> InstantCommand({ climber.setVoltage(12.0.volts) })
                    else -> Commands.none()
                }
            )}))
                .onFalse(InstantCommand({ climber.setVoltage(0.0.volts) }))

        controller.povDown().onTrue(Commands.runOnce({
            scheduleCMD(
                when (stateMachine.getCurrentState()) {
                    States.ClimbState -> InstantCommand({ climber.setVoltage(-12.0.volts) })
                    else -> Commands.none()
                }
            )}))
            .onFalse(InstantCommand({ climber.setVoltage(0.0.volts) }))

        // POV down
//        controller.povDown().onTrue(
//            Commands.sequence(
//                enableAlgaeIntake(3.0.volts),
//                setPoseCommand(ArmPoses.AlgaeFloorIntake.pose, ArmOrders.EWJ.order, 1),
//                disableAlgaeIntake(),
//                Commands.runOnce({ stateMachine.changeState(States.AlgaeState) })
//            )
//        )

        // POV right
        //controller.povRight().onTrue(Commands.none()
//            Commands.sequence(
//                enableAlgaeIntake(3.0.volts),
//                setPoseCommand(ArmPoses.Processor.pose, ArmOrders.EWJ.order, 1),
//                disableAlgaeIntake(),
//                Commands.runOnce({ stateMachine.changeState(States.AlgaeState) })
//            )
        //)

        // POV up
        //controller.povUp().onTrue(setPoseCommand(PoseCommands.Passive))

        controller.leftBumper().onTrue(Commands.runOnce({
            scheduleCMD(ParallelCommandGroup(enableCoralOuttake(), enableAlgaeOuttake()))
            })
        )
            .onFalse(Commands.runOnce({scheduleCMD(ParallelCommandGroup(disableCoralIntake(), disableAlgaeIntake()))}))
    }

}