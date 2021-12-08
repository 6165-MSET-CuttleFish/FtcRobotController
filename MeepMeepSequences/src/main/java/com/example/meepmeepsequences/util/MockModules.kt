package com.example.meepmeepsequences.util

class Intake : Module() {
    fun setPower(power: Double) {
        println("Intake Power $power")
    }
}

class Deposit : Module() {
    private val platform = Platform()
    enum class State {
        LEVEL3,
        LEVEL2,
        IDLE
    }
    fun setState(state: State) {
        println("Lift State $state")
    }
    fun dump() = platform.dump()
}
class Detector {
    enum class Location {
        LEFT, MIDDLE, RIGHT
    }
}

class Platform : Module() {
    fun dump() {
        println("Dump Freight")
    }
}

class Capstone : Module() {
    fun pickUp() {
        println("Capstone retrieved")
    }
    fun ready() {
        println("Capstone picked up")
    }
}

class Carousel : Module() {
    fun on() {
        println("Carousel On")
    }
    fun off() {
        println("Carousel Off")
    }
}

abstract class Module {
    fun isDoingWork(): Boolean {
        return true;
    }
}
