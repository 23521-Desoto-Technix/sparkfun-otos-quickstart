package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt


@Config
class Bot() {
    private lateinit var telem: Telemetry
    private lateinit var hwmap: HardwareMap
    private lateinit var localizer: SparkFunOTOSDrive
    private lateinit var frontLeft: DcMotor
    private lateinit var backLeft: DcMotor
    private lateinit var frontRight: DcMotor
    private lateinit var backRight: DcMotor
    private val xController = PIDController(0.1, 0.0, 0.001)
    private val yController = PIDController(0.1, 0.0, 0.001)
    private val thetaController = PIDController(1.0, 0.0, 0.0)
    var busy = false
    var xTarget: Double = 0.0
    var yTarget: Double = 0.0
    var thetaTarget: Double = 0.0
    var posrange = 0.0
    var velrange = 0.0
    //var slidervalue = null

    //TODO rewrite this to be non blocking.
    fun init(hwmap: HardwareMap, telem: Telemetry) {
        this.hwmap = hwmap
        this.telem = telem
        localizer = SparkFunOTOSDrive(hwmap, Pose2d(0.0, 0.0, 0.0))
        frontLeft = hwmap.dcMotor["frontLeft"]
        backLeft = hwmap.dcMotor["backLeft"]
        frontRight = hwmap.dcMotor["frontRight"]
        backRight = hwmap.dcMotor["backRight"]
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE
    }
    fun toState(
        x: Double? = null,
        y: Double? = null,
        theta: Double? = null,
        posrange: Double = 1.0,
        velrange: Double = 100.0,
        //slidervalue: Double? = null,

    ) {
        xTarget = x ?: xTarget
        yTarget = y ?: yTarget
        thetaTarget = theta ?: thetaTarget
        this.posrange = posrange
        this.velrange = velrange
        //this.slidervalue = slidervalue
        update()
        busy = true
    }
    fun lockPosition() {
        //TODO
        localizer.updatePoseEstimate()
        val x = localizer.pose.position.x
        val y = localizer.pose.position.y
        val theta = localizer.pose.heading.toDouble()
        toState(x, y, theta, 0.0)
    }
    fun drive(x: Double, y: Double, theta: Double) {
        localizer.updatePoseEstimate()
        val botHeading = localizer.pose.heading.toDouble()

        val rotX = x * cos(-botHeading) - y * sin(-botHeading)
        val rotY = x * sin(-botHeading) + y * cos(-botHeading)

        frontLeft.power = rotX + rotY + theta
        backLeft.power = rotX - rotY + theta
        frontRight.power = rotX - rotY - theta
        backRight.power = rotX + rotY - theta
    }
    private fun shortestAngularError(currentAngle: Double, targetAngle: Double): Double {
        val error = targetAngle - currentAngle
        val wrappedError = (error + PI) % (2 * PI) - PI
        return wrappedError
    }
    fun update(runauto: Boolean = true) {
        localizer.updatePoseEstimate()
        val xError = xTarget.minus(localizer.pose.position.x)
        val yError = yTarget.minus(localizer.pose.position.y)
        val thetaError =
            shortestAngularError(localizer.pose.heading.toDouble(), thetaTarget)
        //val thetaError = theta?.minus(localizer.pose.heading.toDouble()) ?: 0.0
        var xPower = xController.calculate(xError)
        var yPower = yController.calculate(yError)
        if (Vector2d(xError, yError).norm() < posrange/2) {
            if (xPower > 0 && xPower < 0.02) {
                xPower = 0.02
            }
            if (xPower < 0 && xPower > -0.02) {
                xPower = -0.02
            }
            if (yPower > 0 && yPower < 0.02) {
                yPower = 0.02
            }
            if (yPower < 0 && yPower > -0.02) {
                yPower = -0.02
            }
        }
        val thetaPower = thetaController.calculate(thetaError)
        //val thetaPower = 0.0
        if (runauto) {
            drive(xPower, yPower, thetaPower)
        } else {
            //drive(0.0, 0.0, 0.0)
        }
        val normalizedError = sqrt(abs(xError * xError) + abs(yError + yError))
        val vel = localizer.pose.minus(localizer.poseHistory.takeLast(2).first()).line.norm()
        if (normalizedError < posrange ) {
            busy = false
            drive(0.0,0.0,0.0)
            telem.addData("Done", normalizedError < posrange)
            telem.update()
        }
        if (normalizedError < posrange/2 && vel < velrange/2) {
            drive(0.0, 0.0, 0.0)
        }
        telem.addData("X", localizer.pose.position.x)
        telem.addData("Y", localizer.pose.position.y)
        telem.addData("Vel", vel)
        telem.addData("Total error", normalizedError)
        telem.addData("Done", normalizedError < posrange)
        telem.update()
    }
}
