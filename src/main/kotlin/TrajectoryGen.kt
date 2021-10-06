import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import kotlin.math.PI
import kotlin.math.roundToInt

object TrajectoryGen {
    val isAutoPaths = true


    //true if using auto paths, false if using teleop paths


    val Double.toRadians get() = Math.toRadians(this)
    val Int.toRadians get() = (this.toDouble().toRadians)


    private val driveConstraints = DriveConstraints(60.0, 60.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 16.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    class TemplateDetector() {
        enum class PipelineResult() {
            //TODO: insert pipelineresults here
            LEFT;
        }
    }

    class MainTeleOp() {
        enum class TemplateState() {
            //TODO: insert states here
            INTAKE;
        }
    }

    class Bot() {
        val roadRunner: RRMecanumDrive = RRMecanumDrive()
        val templateSubsystem = TemplateSubsystem()
        class TemplateSubsystem{
            fun operateSlides(x: Double){}
        }
    }

    class RRMecanumDrive() {
        fun trajectoryBuilder(startPose: Pose2d, startTangent: Double): TrajectoryBuilder {
            return TrajectoryBuilder(startPose, startTangent, combinedConstraints)
        }

        fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder {
            return trajectoryBuilder(startPose, startPose.heading)
        }

    }
    object AutoPaths {
        // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart

        sealed class AutoPathElement(open val name: String) {
            class Path(override val name: String, val trajectory: Trajectory) : AutoPathElement(name)

            //AutoPathElement.Path(name, trajectory)
            class Action(override val name: String, val runner: () -> Unit) : AutoPathElement(name)
            //AutoPathElement.Action(name) {actions to take(include sleeps)}
        }

        val bot: Bot = Bot()
        val drive: RRMecanumDrive = bot.roadRunner
        private fun Pose2d.reverse() = copy(heading = heading + PI)
        private var lastPosition: Pose2d = Pose2d()

        fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path {
            lastPosition = trajectory.end()
            return AutoPathElement.Path(name, trajectory)
            //Start of list of trajectories should not be lastPosition
        }

        //Probably won't be used, but here just in case
        fun makeAction(name: String, action: () -> Unit): AutoPathElement.Action {
            return AutoPathElement.Action(name, action)
            //Redundant but conforms to naming scheme
        }

        // Kotlin 1.3 does not support inline instantiation of SAM interfaces
        class MarkerCallbackImpl(val func: () -> Unit) : MarkerCallback {
            override fun onMarkerReached() = func()
        }
        //TODO: solve this
//    private fun turn(from: Double, to: Double): AutoPathElement.Action {
//        return AutoPathElement.Action("Turn from ${Math.toDegrees(from).roundToInt()}deg" +
//                "to ${Math.toDegrees(to).roundToInt()}deg") {
//            bot.roadRunner.turn(to - from)
//        }
//    }
        val displaySet = TemplateDetector.PipelineResult.LEFT

        //Paste stuff from AutoPaths here ==========================================================================
        //==========================================================================
        //==========================================================================
        //==========================================================================

        //TODO: Insert pose/vector vals here

        //                                                                  ===================================================

        //example
        // private val dropSecondWobble = mapOf(
        //            0 to Pose2d(-4.2 + 1.5, -48.0 - 3.056 + 1f, (-90.0 + 30.268).toRadians),
        //            1 to Pose2d(24.0 - 9.45428 + 3f, -24.0 - 25.16465, (102.4 - 90.0).toRadians),
        //            4 to Pose2d(48 - 5.1, -48.0 - 3.0556 - 3f, (-90.0 + 30.268).toRadians)
        //    )

        val startPose = Pose2d(-48.0 - 24 + 9, -34.0, 180.0.toRadians)

        //TODO: Make Trajectories in trajectorySets

        //                                                                              ====================================================
        private val trajectorySets: Map<TemplateDetector.PipelineResult, List<AutoPathElement>> = mapOf(
            //use !! when accessing maps ie: dropSecondWobble[0]!!
            //example
            TemplateDetector.PipelineResult.LEFT to run {
                val specificPose = Pose2d()
                listOf(
                    makePath(
                        "part 1",
                        drive.trajectoryBuilder(startPose)
                            .back(10.0)
                            .build()
                    ),

                    makeAction("intake something") {
                        Thread.sleep(1000)
                    },

                    makePath(
                        "part 2",
                        drive.trajectoryBuilder(lastPosition)
                            .forward(20.0)
                            .build()
                    )
                )
            }
//
        )

        //end paste  ==========================================================================

        fun createTrajectory(): ArrayList<Trajectory> {
            val list = ArrayList<Trajectory>()

            for (trajectory in trajectorySets[displaySet]!!) {
                if (trajectory is AutoPathElement.Path)
                    list.add(trajectory.trajectory)
            }

            return list
        }

        fun drawOffbounds() {
            GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
        }
    }

    object TeleOpPaths{

        sealed class TeleOpPathElement(open val name: String) {
            class Path(override val name: String, val trajectory: Trajectory): TeleOpPathElement(name){
                fun getTrajPose(percent: Double): Pose2d{
                    return trajectory[percent.coerceAtMost(100.0) / 100 * trajectory.duration()]
                }
            }
            //AutoPathElement.Path(name, trajectory)
            class Action(override val name: String, val runner: (Double) -> () -> Unit): TeleOpPathElement(name){
                fun getRunner(percent: Double): () -> Unit{
                    return runner(percent)
                }
            }
            //AutoPathElement.Action(name) {percent: Double -> {actions to take(no sleeps, divide by percent)}}
            class ActionPath(override val name: String, val trajectory: Trajectory, val runner: (Double) -> () -> Unit): TeleOpPathElement(name){
                fun getRunnerPath(percent: Double): Pair<Trajectory, () -> Unit>{
                    return Pair(trajectory, getRunner(percent))
                }
                fun getRunnerPose(percent: Double): Pair<Pose2d, () -> Unit> {
                    return Pair(getTrajPose(percent), getRunner(percent))
                }
                fun getRunner(percent: Double): () -> Unit{
                    return runner(percent)
                }
                fun getTrajPose(percent: Double): Pose2d{
                    return trajectory[percent.coerceAtMost(100.0) / 100 * trajectory.duration()]
                }
            }
            //AutoPathElement.ActionPath(name, trajectory) {percent: Double -> {Actions to take(no sleeps, divide by percent)}}
        }

        val bot: Bot = Bot()
        val drive: RRMecanumDrive = bot.roadRunner
        private fun Pose2d.reverse() = copy(heading = heading + PI)
        private var lastPosition: Pose2d = Pose2d()

        fun makePath(name: String, trajectory: Trajectory): TeleOpPathElement.Path{
            lastPosition = trajectory.end()
            return TeleOpPathElement.Path(name, trajectory)
            //Start of list of trajectories should not be lastPosition
        }

        //Probably won't be used, but here just in case
        fun makeAction(name: String, action: (Double) -> () -> Unit): TeleOpPathElement.Action{
            return TeleOpPathElement.Action(name, action)
            //Redundant but conforms to naming scheme
        }

        fun makeActionPath(name: String, trajectory: Trajectory, action: (Double) -> () -> Unit): TeleOpPathElement.ActionPath{
            return TeleOpPathElement.ActionPath(name, trajectory, action)
        }

        // Kotlin 1.3 does not support inline instantiation of SAM interfaces
        class MarkerCallbackImpl(val func: () -> Unit) : MarkerCallback {
            override fun onMarkerReached() = func()
        }
        //TODO: solve this
//    private fun turn(from: Double, to: Double): AutoPathElement.Action {
//        return AutoPathElement.Action("Turn from ${Math.toDegrees(from).roundToInt()}deg" +
//                "to ${Math.toDegrees(to).roundToInt()}deg") {
//            bot.roadRunner.turn(to - from)
//        }
//    }


        val displaySet = MainTeleOp.TemplateState.INTAKE

        //Paste stuff from TeleOpPaths here ==========================================================================
        //==========================================================================
        //==========================================================================
        //==========================================================================

        //TODO: Insert pose/vector vals here

        //                                                                  ===================================================

        //example
        // private val dropSecondWobble = mapOf(
        //            0 to Pose2d(-4.2 + 1.5, -48.0 - 3.056 + 1f, (-90.0 + 30.268).toRadians),
        //            1 to Pose2d(24.0 - 9.45428 + 3f, -24.0 - 25.16465, (102.4 - 90.0).toRadians),
        //            4 to Pose2d(48 - 5.1, -48.0 - 3.0556 - 3f, (-90.0 + 30.268).toRadians)
        //    )

        val startPose = Pose2d(-48.0 - 24 + 9, -34.0, PI)

        //TODO: insert action vals here

        //                                                                  =======================================================

        //example
        //private val shootRings = AutoPathElement.Action("Shoot 3 rings") {
        //        bot.shooter.shootRings(opMode, 3, 0.8)
        //        bot.shooter.turnOff()
        //        Thread.sleep(1000)
        //    }


        //TODO: Make Trajectories

        //                                                                              ====================================================
        private val trajectorySets: Map<MainTeleOp.TemplateState, List<TeleOpPathElement>> = mapOf(
            //use !! when accessing maps ie: dropSecondWobble[0]!!
            //example
            MainTeleOp.TemplateState.INTAKE to run{
                val specificPose = Pose2d()
                listOf(
                    makePath("part 1",
                        drive.trajectoryBuilder(startPose)
                            .forward(10.0)
                            .build()),

                    makeAction("do something") {percent: Double ->
                        {bot.templateSubsystem.operateSlides(4 * percent)}
                    },

                    makeActionPath("part 2",
                        drive.trajectoryBuilder(lastPosition)
                            .forward(10.0)
                            .build()) {percent: Double ->
                        {bot.templateSubsystem.operateSlides(4 - 4*percent)}
                    }
                )
            }
//
        )

        //end paste  ==========================================================================

        fun createTrajectory(): ArrayList<Trajectory> {
            val list = ArrayList<Trajectory>()

            for (trajectory in trajectorySets[displaySet]!!) {
                if (trajectory is TeleOpPathElement.Path)
                    list.add(trajectory.trajectory)
                else if (trajectory is TeleOpPathElement.ActionPath)
                    list.add(trajectory.trajectory)
            }

            return list
        }

        fun drawOffbounds() {
            GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
        }
    }

}

