#include "3rdParty/render_progress.h"

#include <iomanip>
#include <iostream>
#include <sstream>

namespace COR {

    std::string format_hhmmss(double seconds) {
        if (seconds < 0) seconds = 0;

        long long s = (long long)(seconds + 0.5);
        long long h = s / 3600; s %= 3600;
        long long m = s / 60;   s %= 60;

        std::ostringstream oss;
        oss << std::setfill('0')
            << std::setw(2) << h << ":"
            << std::setw(2) << m << ":"
            << std::setw(2) << s;
        return oss.str();
    }

    RenderProgress::RenderProgress(int w, int h, int spp)
        : W(w), H(h), SPP(spp) {
        totalSamples = (long long)W * (long long)H * (long long)SPP;
        t0 = std::chrono::steady_clock::now();
        lastPrint = t0;
    }

    void RenderProgress::addSamples(long long n) {
        doneSamples += n;

        auto now = std::chrono::steady_clock::now();
        double dtSincePrint = std::chrono::duration<double>(now - lastPrint).count();
        if (dtSincePrint < 0.25) return; // print 4 times/sec max
        lastPrint = now;

        double elapsed = std::chrono::duration<double>(now - t0).count();
        double rate = (elapsed > 0.0) ? (double)doneSamples / elapsed : 0.0;
        double remaining = (rate > 0.0) ? (double)(totalSamples - doneSamples) / rate : 0.0;

        double pct = 100.0 * (double)doneSamples / (double)totalSamples;

        std::cout << "\r"
            << "Progress: " << std::fixed << std::setprecision(1) << pct << "% "
            << "(" << doneSamples << "/" << totalSamples << " samples) "
            << "Elapsed " << format_hhmmss(elapsed) << " "
            << "ETA " << format_hhmmss(remaining)
            << std::flush;
    }

    void RenderProgress::done() {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - t0).count();
        std::cout << "\r"
            << "Progress: 100.0% "
            << "(" << totalSamples << "/" << totalSamples << " samples) "
            << "Elapsed " << format_hhmmss(elapsed) << " "
            << "ETA 00:00:00"
            << "          \n";
    }

}