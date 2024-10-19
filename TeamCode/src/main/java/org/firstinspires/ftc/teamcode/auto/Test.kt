package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
@Autonomous
class Test : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val bot = Bot()
        bot.init(hardwareMap, telemetry)
        while (opModeInInit() && !isStopRequested) {
            bot.update()
        }
        waitForStart()
        bot.toState(0.0,40.0,Math.toRadians(90.0),0.0, 0.0)
    }
}
