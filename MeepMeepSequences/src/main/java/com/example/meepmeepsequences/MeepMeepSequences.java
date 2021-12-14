package com.example.meepmeepsequences;

import com.example.meepmeepsequences.util.Context;
import com.example.meepmeepsequences.util.Detector;

public class MeepMeepSequences {
    public static void main(String[] args) {
        Context.location = Detector.Location.RIGHT;
        new BasicCarouselPath().carouselPath(false).start();
    }
}