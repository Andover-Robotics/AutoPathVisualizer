import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator
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


    val Double.toRadians get() = (Math.toRadians(this))

    private val driveConstraints = DriveConstraints(60.0, 60.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 16.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    class GlobalConfig{
        val alliance: Alliance = Alliance.RED

        enum class Alliance{
            BLUE,
            RED
        }

    }

    class TemplateDetector() {
        enum class PipelineResult() {
            //TODO: insert pipelineresults here
            LEFT,
            MIDDLE,
            RIGHT
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
        fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean): TrajectoryBuilder {
            return TrajectoryBuilder(startPose, reversed, combinedConstraints)
        }

        fun trajectoryBuilder(startPose: Pose2d, startTangent: Double): TrajectoryBuilder {
            return TrajectoryBuilder(startPose, startTangent, combinedConstraints)
        }

        fun trajectoryBuilder(trajectory: Trajectory, time: Double): TrajectoryBuilder {
            return TrajectoryBuilder(trajectory, time, combinedConstraints)
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


        val Double.toRadAS get() = (if(globalConfig.alliance == GlobalConfig.Alliance.RED) this.toRadians else -this.toRadians)
        val Int.toRadians get() = (this.toDouble().toRadians)
        val Int.toRadAS get() = (this.toDouble().toRadAS)

        enum class AutoType{
            PARK,
            CAROUSEL,
            TESTING
        }

        fun p2d(x: Double, y: Double, h: Double): Pose2d{
            return Pose2d(x, if(globalConfig.alliance == GlobalConfig.Alliance.RED) y else -y, if(globalConfig.alliance == GlobalConfig.Alliance.RED) h else -h)
        }
        fun p2d(v: Vector2d, h: Double): Pose2d{
            return Pose2d(v.x, if(globalConfig.alliance == GlobalConfig.Alliance.RED) v.y else -v.y, if(globalConfig.alliance == GlobalConfig.Alliance.RED) h else -h)
        }
        fun v2d(x: Double, y: Double): Vector2d{
            return Vector2d(x, if(globalConfig.alliance == GlobalConfig.Alliance.RED) y else -y)
        }
        val bypassSize = 10
        fun bypass(startPose: Pose2d, endPose: Pose2d): Pose2d{
            val dif = endPose.minus(startPose).vec()
            return Pose2d(startPose.vec().plus(dif.div(dif.norm() * bypassSize)), startPose.heading)
        }
        fun bypassVec(startVec: Vector2d, endVec: Vector2d): Vector2d{
            val dif = endVec.minus(startVec)
            return startVec.plus(dif.div(dif.norm() * bypassSize))
        }
        fun bypassStraight(startPose: Pose2d, angle: Double): Pose2d{
            return Pose2d(startPose.vec().plus(startPose.vec().rotated(angle - 90)), startPose.heading)
        }

        val globalConfig: GlobalConfig = GlobalConfig()

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
        val autoType = AutoType.CAROUSEL
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


        val startPose = p2d(-36.0, -72.0+9.0, -90.0.toRadians)

        //TODO: Make Trajectories in trajectorySets

        val dropFreight = mapOf(
            TemplateDetector.PipelineResult.LEFT to listOf(
                makeAction("raiseArm"){
//                bot.outtake.armHigh();
                },
                makePath("drive to drop",
                    drive.trajectoryBuilder(startPose, 90.0.toRadAS)
                        .splineTo(v2d(-24.0, -38.0), 60.0.toRadAS)
                        .build()),
                makeAction("drop"){
//                bot.outtake.bucketDrop();
                }
            ),
            TemplateDetector.PipelineResult.RIGHT to listOf(
                makeAction("test"){

                }
            ),
            TemplateDetector.PipelineResult.MIDDLE to listOf(
                makeAction("test"){

                }
            )
        )

        val dropFreightPose = mapOf(
            TemplateDetector.PipelineResult.LEFT to getLastPose(dropFreight[TemplateDetector.PipelineResult.LEFT]!!),
            TemplateDetector.PipelineResult.RIGHT to getLastPose(dropFreight[TemplateDetector.PipelineResult.RIGHT]!!),
            TemplateDetector.PipelineResult.MIDDLE to getLastPose(dropFreight[TemplateDetector.PipelineResult.MIDDLE]!!)
        )

        fun getLastPose(paths: List<AutoPathElement>): Pose2d {
            for(i in paths.reversed()){
                if(i is AutoPathElement.Path){
                    return i.trajectory.end()
                }
            }
            return startPose
        }

        fun park(result: TemplateDetector.PipelineResult): List<AutoPathElement>{
            return run{
                dropFreight[result]!! + listOf(
                    makePath("drive into warehouse",
                        drive.trajectoryBuilder(dropFreightPose[result]!!)
                            .forward(72.0)
                            .build())
                )
            }
        }

        fun carousel(result: TemplateDetector.PipelineResult): List<AutoPathElement> {
            return run {
                dropFreight[result]!! + listOf(
                    makePath("drive to carousel",
                        drive.trajectoryBuilder(dropFreightPose[result]!!)
                            .splineToSplineHeading(p2d(-59.0, -59.0, -135.toRadians), -135.toRadAS)
//                            .addSpatialMarker(v2d(-55.0, -55.0)){
//                                bot.carousel.run()
//                            }
                            .build()
                    ),
                    makeAction("do carousel"){
                        Thread.sleep(2000)
//                        bot.carousel.stop()
                    },
                    makePath("park",
                        drive.trajectoryBuilder(lastPosition, 45.toRadAS)
                            .splineToSplineHeading(p2d(-72.0 + 9.0, -36.0,  0.0), 180.toRadAS)
                            .build()
                    )
                )
            }
        }

        private val trajectorySets: Map<AutoType, Map<TemplateDetector.PipelineResult, List<AutoPathElement>>> = mapOf(
            AutoType.PARK to mapOf(
                TemplateDetector.PipelineResult.LEFT to park(TemplateDetector.PipelineResult.LEFT),
                TemplateDetector.PipelineResult.RIGHT to park(TemplateDetector.PipelineResult.RIGHT),
                TemplateDetector.PipelineResult.MIDDLE to park(TemplateDetector.PipelineResult.MIDDLE)
            ),
            AutoType.CAROUSEL to mapOf(
                TemplateDetector.PipelineResult.LEFT to carousel(TemplateDetector.PipelineResult.LEFT),
                TemplateDetector.PipelineResult.RIGHT to carousel(TemplateDetector.PipelineResult.RIGHT),
                TemplateDetector.PipelineResult.MIDDLE to carousel(TemplateDetector.PipelineResult.MIDDLE)
            ),
            AutoType.TESTING to mapOf(
                TemplateDetector.PipelineResult.MIDDLE to run{
                    listOf(
                        makePath("do stuff",
                            drive.trajectoryBuilder(startPose)
                                .forward(20.0)
                                .build()
                        )
                    )
                },
                TemplateDetector.PipelineResult.LEFT to run{
                    listOf(
                        makePath("forward 8",
                            drive.trajectoryBuilder(startPose)
                                .lineToConstantHeading(v2d(0.0, 24.0))
                                .splineToConstantHeading(v2d(0.001, 24.0), 0.0)
                                .lineToConstantHeading(v2d(24.0, 24.0))
                                .build())
                    )
                },
                TemplateDetector.PipelineResult.RIGHT to run{
                    val c1 = Pose2d()
                    val c2 = p2d(0.0, 24.0, 90.0.toRadians)
                    val c3 = p2d(24.0, 24.0, 180.0.toRadians)
                    val c4 = p2d(24.0, 0.0, -90.0.toRadians)
                    listOf(
//                        makeAction("set pose"){
//                            drive.poseEstimate = Pose2d()
//                        },
                        makePath("testing",
                            drive.trajectoryBuilder(c1)
                                .lineToConstantHeading(c2.vec())
                                .splineToConstantHeading(bypassVec(c2.vec(), c3.vec()), 0.0)
                                .lineToConstantHeading(c3.vec())
                                .splineToConstantHeading(bypassVec(c3.vec(), c4.vec()), -90.0.toRadAS)
                                .lineToConstantHeading(c4.vec())
                                .splineToConstantHeading(bypassVec(c4.vec(), c1.vec()), 180.0.toRadAS)
                                .lineToConstantHeading(c1.vec())
                                .splineToConstantHeading(bypassVec(c1.vec(), c3.vec()), 45.0.toRadAS)
                                .lineToSplineHeading(c3)
                                .splineToConstantHeading(bypassVec(c3.vec(), c4.vec()), -90.0.toRadAS)
                                .lineToSplineHeading(c4)
                                .splineToConstantHeading(bypassVec(c4.vec(), c2.vec()), 135.0.toRadAS)
                                .lineToSplineHeading(c2)
                                .splineToConstantHeading(bypassVec(c2.vec(), c1.vec()), -90.0.toRadAS)
                                .lineToSplineHeading(c1)
                                .splineToSplineHeading(c4, 0.0)
                                .splineToSplineHeading(c3, 90.0.toRadAS)
                                .splineToSplineHeading(c2, 180.0.toRadAS)
                                .splineToSplineHeading(c1, -90.0.toRadAS)
                                .build()
                        )
                    )
                }
            )
        )

        //end paste  ==========================================================================

        fun createTrajectory(): ArrayList<Trajectory> {
            val list = ArrayList<Trajectory>()

            for (trajectory in trajectorySets[autoType]!![displaySet]!!) {
                if (trajectory is AutoPathElement.Path)
                    list.add(trajectory.trajectory)
            }

            return list
        }

        fun drawOffbounds() {
            GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
        }
    }


}

