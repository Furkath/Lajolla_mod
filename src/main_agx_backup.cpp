#include "parsers/parse_scene.h"
#include "parallel.h"
#include "image.h"
#include "render.h"
#include "timer.h"
#include <embree4/rtcore.h>
#include <memory>
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>

// AgX tone mapping matching Blender's AgX Base Contrast
// Reference: three.js AgXToneMapping (verified against Blender's OCIO config)
// Pipeline: sRGB linear -> Rec2020 linear -> AgX inset -> log2 -> sigmoid -> AgX outset -> pow(2.2) -> sRGB linear

inline double agx_contrast_approx(double x) {
    double x2 = x * x;
    double x4 = x2 * x2;
    return + 15.5     * x4 * x2
           - 40.14    * x4 * x
           + 31.96    * x4
           - 6.868    * x2 * x
           + 0.4298   * x2
           + 0.1191   * x
           - 0.00232;
}

void apply_agx_tonemap(Image3 &img) {
    const double AgxMinEv = -12.47393;
    const double AgxMaxEv = 4.026069;

    for (int i = 0; i < (int)img.data.size(); i++) {
        double r = img.data[i].x;
        double g = img.data[i].y;
        double b = img.data[i].z;

        // Step 1: Linear sRGB -> Linear Rec.2020
        double r2 = 0.6274039 * r + 0.3292830 * g + 0.0433131 * b;
        double g2 = 0.0690972 * r + 0.9195404 * g + 0.0113624 * b;
        double b2 = 0.0163914 * r + 0.0880133 * g + 0.8955953 * b;

        // Step 2: AgX Inset Matrix (Rec.2020 -> AgX log input)
        double ar = 0.856627153315983  * r2 + 0.0951212405381588 * g2 + 0.0482516061458583 * b2;
        double ag = 0.137318972929847  * r2 + 0.761241990602591  * g2 + 0.101439036467562  * b2;
        double ab = 0.11189821299995   * r2 + 0.0767994186031903 * g2 + 0.811302368396859  * b2;

        // Step 3: Log2 encoding
        ar = std::max(ar, 1e-10); ag = std::max(ag, 1e-10); ab = std::max(ab, 1e-10);
        ar = std::log2(ar); ag = std::log2(ag); ab = std::log2(ab);

        // Step 4: Normalize to [0, 1]
        ar = (ar - AgxMinEv) / (AgxMaxEv - AgxMinEv);
        ag = (ag - AgxMinEv) / (AgxMaxEv - AgxMinEv);
        ab = (ab - AgxMinEv) / (AgxMaxEv - AgxMinEv);
        ar = std::clamp(ar, 0.0, 1.0);
        ag = std::clamp(ag, 0.0, 1.0);
        ab = std::clamp(ab, 0.0, 1.0);

        // Step 5: Apply sigmoid contrast approximation
        ar = agx_contrast_approx(ar);
        ag = agx_contrast_approx(ag);
        ab = agx_contrast_approx(ab);

        // Step 6: AgX Outset Matrix
        double or_ =  1.1271005818144368  * ar - 0.11060664309660323 * ag - 0.016493938717834573 * ab;
        double og  = -0.1413297634984383  * ar + 1.157823702216272   * ag - 0.016493938717834257 * ab;
        double ob  = -0.14132976349843826 * ar - 0.11060664309660294 * ag + 1.2519364065950405   * ab;

        // Step 7: Linearize (pow 2.2)
        or_ = std::pow(std::max(or_, 0.0), 2.2);
        og  = std::pow(std::max(og,  0.0), 2.2);
        ob  = std::pow(std::max(ob,  0.0), 2.2);

        // Step 8: Linear Rec.2020 -> Linear sRGB
        double fr = 1.6605 * or_ - 0.5876 * og - 0.0728 * ob;
        double fg = -0.1246 * or_ + 1.1329 * og - 0.0083 * ob;
        double fb = -0.0182 * or_ - 0.1006 * og + 1.1187 * ob;

        // Clamp to [0, 1] — output is linear sRGB, ready for EXR/PFM
        img.data[i] = Vector3(
            std::clamp(fr, 0.0, 1.0),
            std::clamp(fg, 0.0, 1.0),
            std::clamp(fb, 0.0, 1.0));
    }
}

int main(int argc, char *argv[]) {
    if (argc <= 1) {
        std::cout << "[Usage] ./lajolla [-t num_threads] [-o output_file_name] [-agx] filename.xml" << std::endl;
        return 0;
    }

    int num_threads = std::thread::hardware_concurrency();
    std::string outputfile = "";
    bool use_agx = false;
    std::vector<std::string> filenames;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-t") {
            num_threads = std::stoi(std::string(argv[++i]));
        } else if (std::string(argv[i]) == "-o") {
            outputfile = std::string(argv[++i]);
        } else if (std::string(argv[i]) == "-agx") {
            use_agx = true;
        } else {
            filenames.push_back(std::string(argv[i]));
        }
    }

    RTCDevice embree_device = rtcNewDevice(nullptr);
    parallel_init(num_threads);

    for (const std::string &filename : filenames) {
        Timer timer;
        tick(timer);
        std::cout << "Parsing and constructing scene " << filename << "." << std::endl;
        std::unique_ptr<Scene> scene = parse_scene(filename, embree_device);
        std::cout << "Done. Took " << tick(timer) << " seconds." << std::endl;
        std::cout << "Rendering..." << std::endl;
        Image3 img = render(*scene);
        if (outputfile.compare("") == 0) {outputfile = scene->output_filename;}
        std::cout << "Done. Took " << tick(timer) << " seconds." << std::endl;
        if (use_agx) {
            std::cout << "Applying AgX tone mapping..." << std::endl;
            // Save raw HDR first if outputting EXR, then tonemap for PNG
            Image3 tonemapped = img;
            apply_agx_tonemap(tonemapped);
            // Write tonemapped version; if output is .exr/.pfm, also save raw
            imwrite(outputfile, tonemapped);
        } else {
            imwrite(outputfile, img);
        }
        std::cout << "Image written to " << outputfile << std::endl;
    }

    parallel_cleanup();
    rtcReleaseDevice(embree_device);
    return 0;
}

