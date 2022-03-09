package com.example.meepmeepsequences

import com.example.meepmeepsequences.util.Context
import com.example.meepmeepsequences.util.Context.windowSize
import com.example.meepmeepsequences.util.Detector
import com.example.meepmeepsequences.util.addMultiPath
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background

object MeepMeepSequences {
    @JvmStatic
    fun main(args: Array<String>) {
        Context.location = Detector.Location.RIGHT
        MeepMeep(windowSize)
            .setBackground(Background.FIELD_FREIGHTFRENZY_ADI_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addMultiPath(BasicPaths()::carouselPath)
            .start()
    }
}