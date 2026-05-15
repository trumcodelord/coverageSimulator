#include "hud_renderer.h"

#include "grid.h"
#include "visual_layout.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

namespace
{
    string hudState = "NONE";

    Scalar hudStateColor(const string &state)
    {
        if (state == "NONE") return Scalar(60, 60, 60);
        if (state == "NORMAL" || state == "FIND") return Scalar(0, 140, 255);
        if (state == "ALERT" || state == "RUN!") return Scalar(0, 180, 0);

        if (state == "HOLD_SAFE" ||
            state == "WAIT" ||
            state == "WAIT_FOR_COMMAND")
        {
            return Scalar(0, 0, 220);
        }

        if (state == "RETURN_TO_BASE" ||
            state == "RETURN_WAIT" ||
            state == "RETURN_DETOUR")
        {
            return Scalar(0, 165, 255);
        }

        if (state == "RECHARGING") return Scalar(180, 80, 0);
        if (state == "POWER_SAVE") return Scalar(120, 120, 120);

        if (state == "STOP" ||
            state == "FINAL_PUSH" ||
            state == "POWER_LOSS")
        {
            return Scalar(0, 0, 255);
        }

        if (state == "DONE") return Scalar(0, 160, 0);

        return Scalar(60, 60, 60);
    }

    bool fitHUDInBox(
        const vector<string> &lines,
        const Rect &box,
        double &fontScale,
        int &lineStep,
        int &thickness
    ) {
        if (box.width <= 0 || box.height <= 0)
            return false;

        const int padding = 10;
        const int fontFace = FONT_HERSHEY_SIMPLEX;

        for (double scale = 0.75; scale >= 0.38; scale -= 0.05)
        {
            int t = (scale >= 0.55) ? 2 : 1;
            int maxTextWidth = 0;
            int maxTextHeight = 0;

            for (const string &line : lines)
            {
                int baseline = 0;

                Size textSize = getTextSize(
                    line,
                    fontFace,
                    scale,
                    t,
                    &baseline
                );

                maxTextWidth = max(maxTextWidth, textSize.width);
                maxTextHeight = max(maxTextHeight, textSize.height + baseline);
            }

            int step = max(16, (int)(maxTextHeight * 1.45));
            int totalHeight = step * (int)lines.size();

            if (maxTextWidth + 2 * padding <= box.width &&
                totalHeight + 2 * padding <= box.height)
            {
                fontScale = scale;
                lineStep = step;
                thickness = t;
                return true;
            }
        }

        return false;
    }

    bool chooseHUDBox(
        const vector<string> &lines,
        Rect &chosenBox,
        double &fontScale,
        int &lineStep,
        int &thickness
    ) {
        Rect grid = visualGridRect();

        int gridLeft = grid.x;
        int gridTop = grid.y;
        int gridRight = grid.x + grid.width;
        int gridBottom = grid.y + grid.height;

        const int gap = 12;
        const int edge = 10;

        vector<Rect> candidates;

        candidates.push_back(Rect(
            gridRight + gap,
            edge,
            max(0, visualCanvasWidth() - gridRight - gap - edge),
            max(0, visualCanvasHeight() - 2 * edge)
        ));

        candidates.push_back(Rect(
            edge,
            edge,
            max(0, gridLeft - gap - edge),
            max(0, visualCanvasHeight() - 2 * edge)
        ));

        candidates.push_back(Rect(
            edge,
            gridBottom + gap,
            max(0, visualCanvasWidth() - 2 * edge),
            max(0, visualCanvasHeight() - gridBottom - gap - edge)
        ));

        candidates.push_back(Rect(
            edge,
            edge,
            max(0, visualCanvasWidth() - 2 * edge),
            max(0, gridTop - gap - edge)
        ));

        for (const Rect &box : candidates)
        {
            if (fitHUDInBox(lines, box, fontScale, lineStep, thickness))
            {
                chosenBox = box;
                return true;
            }
        }

        return false;
    }
}

void setHUDState(const string &state)
{
    hudState = state;
}

void clearHUDState()
{
    hudState = "NONE";
}

void paintHUD(Mat &canvas, const Robot &rb, int delay)
{
    bool hasActivePath = (rb.pathID < (int)rb.path.size());
    int currentPathLength = (int)rb.path.size();

    vector<string> lines;

    lines.push_back("Steps: " + to_string(rb.steps));
    lines.push_back(
        "Energy: " + to_string(rb.energy) + "/" + to_string(rb.maxEnergy)
    );
    lines.push_back("Used: " + to_string(rb.totalEnergyUsed));
    lines.push_back("Returns: " + to_string(rb.returnCount));
    lines.push_back("Recharges: " + to_string(rb.rechargeCount));
    lines.push_back(string("Active path: ") + (hasActivePath ? "YES" : "NO"));
    lines.push_back("Path length: " + to_string(currentPathLength));
    lines.push_back("State: " + hudState);

    Rect hudBox;
    double fontScale = 0.0;
    int lineStep = 0;
    int thickness = 1;

    if (!chooseHUDBox(lines, hudBox, fontScale, lineStep, thickness))
        return;

    const int padding = 10;
    const int fontFace = FONT_HERSHEY_SIMPLEX;

    Point origin(
        hudBox.x + padding,
        hudBox.y + padding + lineStep
    );

    for (int i = 0; i < (int)lines.size(); i++)
    {
        Scalar color = Scalar(40, 40, 40);

        if (i == (int)lines.size() - 1)
            color = hudStateColor(hudState);

        putText(
            canvas,
            lines[i],
            Point(origin.x, origin.y + i * lineStep),
            fontFace,
            fontScale,
            color,
            thickness,
            LINE_AA
        );
    }
}
