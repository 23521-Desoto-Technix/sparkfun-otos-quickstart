package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class LowerLimitTuner : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val bot = Bot()
        bot.init(hardwareMap, telemetry)
        var power = 0.0
        while (!isStopRequested) {
            bot.drive(0.0, power, 0.0)
            if (gamepad1.dpad_up) {
                power += 0.005
            }
            if (gamepad1.dpad_down) {
                power -= 0.005
            }
            telemetry.addData("Power", power)
            telemetry.update()
        }
    }
}