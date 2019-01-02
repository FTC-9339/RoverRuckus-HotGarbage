package com.roguemafia.rogueftc.dash

import android.os.Bundle
import com.qualcomm.ftccommon.FtcRobotControllerService
import com.qualcomm.robotcore.eventloop.EventLoop
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivityCallbacks

class RogueDash {

    private class RogueDashHolder {
        val dashInstance = RogueDash()
    }

    init {
        FtcRobotControllerActivityCallbacks.getInstance().addCreateCallback {onCreate(it)}
        FtcRobotControllerActivityCallbacks.getInstance().addServiceBindCallback { onServiceBind(it) }
        FtcRobotControllerActivityCallbacks.getInstance().addDestroyCallback { onDestroy() }
        FtcRobotControllerActivityCallbacks.getInstance().addAttachEventLoopCallback { attachEventLoop(it) }
    }

    // FtcRobotControllerActivity callbacks.

    private fun onCreate(bundle: Bundle) {

    }

    private fun onServiceBind(service: FtcRobotControllerService) {

    }

    private fun onDestroy() {

    }

    private fun attachEventLoop(eventLoop: EventLoop) {

    }
}