package com.roguemafia.rogueftc.robot

class RogueBot {

    internal constructor() {

    }

    class Builder {
        var isBenchTest: Boolean = false

        fun createRobot(): RogueBot {
            return RogueBot()
        }
    }
}