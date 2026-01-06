// include/util/render_progress.h

#pragma once

#include <chrono>
#include <string>

namespace COR {

    std::string format_hhmmss(double seconds);

    struct RenderProgress {
        int W, H, SPP;
        long long totalSamples;
        long long doneSamples = 0;

        std::chrono::steady_clock::time_point t0;
        std::chrono::steady_clock::time_point lastPrint;

        RenderProgress(int w, int h, int spp);

        void addSamples(long long n);
        void done();
    };

}