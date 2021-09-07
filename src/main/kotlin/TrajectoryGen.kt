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
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(60.0, 60.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 16.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    val displaySet = TemplateDetector.PipelineResult.LEFT

    sealed class AutoPathElement(open val name: String) {
        class Path(override val name: String, val trajectory: Trajectory): AutoPathElement(name)
        //AutoPathElement.Path(name, trajectory)
        class Action(override val name: String, val runner: () -> Unit): AutoPathElement(name)
        //AutoPathElement.Action(name) {actions to take(include sleeps)}
    }

    class TemplateDetector(){
        enum class PipelineResult(){//TODO: insert pipelineresults here
        LEFT;
        }
    }

    class Bot(){
        val roadRunner: RRMecanumDrive = RRMecanumDrive()
    }

    class RRMecanumDrive(){
        fun trajectoryBuilder(startPose: Pose2d, startTangent: Double): TrajectoryBuilder{
            return TrajectoryBuilder(startPose, startTangent, combinedConstraints)
        }
        fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder{
            return trajectoryBuilder(startPose, startPose.heading)
        }

    }

    val bot: Bot = Bot()
    val drive: RRMecanumDrive = bot.roadRunner
    val Double.toRadians get() = Math.toRadians(this)
    val Int.toRadians get() = (this.toDouble().toRadians)
    private fun Pose2d.reverse() = copy(heading = heading + PI)
    private var lastPosition: Pose2d = Pose2d()

    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path{
        lastPosition = trajectory.end()
        return AutoPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    //Probably won't be used, but here just in case
    fun makeAction(name: String, action: () -> Unit): AutoPathElement.Action{
        return AutoPathElement.Action(name, action)
        //Redundant but conforms to naming scheme
    }

    // Kotlin 1.3 does not support inline instantiation of SAM interfaces
    class MarkerCallbackImpl(val func: () -> Unit): MarkerCallback {
        override fun onMarkerReached() = func()
    }
    //TODO: solve this
//    private fun turn(from: Double, to: Double): AutoPathElement.Action {
//        return AutoPathElement.Action("Turn from ${Math.toDegrees(from).roundToInt()}deg" +
//                "to ${Math.toDegrees(to).roundToInt()}deg") {
//            bot.roadRunner.turn(to - from)
//        }
//    }


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
        TemplateDetector.PipelineResult.LEFT to run{
            val specificPose = Pose2d()
            listOf(
                makePath("part 1",
                    drive.trajectoryBuilder(startPose)
                        .back(10.0)
                        .build()),

                makeAction("intake something") {
                    Thread.sleep(1000)
                },

                makePath("part 2",
                    drive.trajectoryBuilder(lastPosition)
                        .forward(10.0)
                        .build())
            )
        }
//
    )

    //end paste  ==========================================================================

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        for(trajectory in trajectorySets[displaySet]!!) {
            if(trajectory is AutoPathElement.Path)
                list.add(trajectory.trajectory)
        }

        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

