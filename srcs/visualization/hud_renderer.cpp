#include "hud_renderer.h"

#include "grid.h"
#include "visual_layout.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <deque>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

namespace
{
    string hudState = "NONE";
    deque<string> hudEvents;

    constexpr int MAX_VISIBLE_HUD_EVENTS = 13;
    constexpr int MAX_STORED_HUD_EVENTS = 60;

    Scalar hudStateColor(const string &state)
    {
        if (state == "NONE") return Scalar(60, 60, 60);
        if (state == "NORMAL" || state == "FIND") return Scalar(0, 140, 255);

        if (state == "ALERT" || state == "RUN!")
            return Scalar(0, 180, 0);

        if (state == "HOLD_SAFE" ||
            state == "WAIT" ||
            state == "WAIT_FOR_COMMAND")
            return Scalar(0, 0, 220);

        if (state == "RETURN_TO_BASE" ||
            state == "RETURN_WAIT" ||
            state == "RETURN_DETOUR" ||
            state == "PARTIAL_RETURNED")
            return Scalar(0, 165, 255);

        if (state == "RECHARGING") return Scalar(180, 80, 0);
        if (state == "POWER_SAVE") return Scalar(120, 120, 120);

        if (state == "STOP" ||
            state == "FINAL_PUSH" ||
            state == "POWER_LOSS")
            return Scalar(0, 0, 255);

        if (state == "DONE") return Scalar(0, 160, 0);

        return Scalar(60, 60, 60);
    }

    string trimEventForHUD(const string &event)
    {
        constexpr int MAX_LEN = 46;

        if ((int)event.size() <= MAX_LEN)
            return event;

        return event.substr(0, MAX_LEN - 3) + "...";
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

        for (double scale = 0.70; scale >= 0.30; scale -= 0.04)
        {
            int t = (scale >= 0.55) ? 2 : 1;
            int maxTextWidth = 0;
            int maxTextHeight = 0;

            for (const string &line : lines)
            {
                int baseline = 0;
                Size textSize = getTextSize(line, fontFace, scale, t, &baseline);

                maxTextWidth = max(maxTextWidth, textSize.width);
                maxTextHeight = max(maxTextHeight, textSize.height + baseline);
            }

            int step = max(13, (int)(maxTextHeight * 1.25));
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

    vector<string> buildHUDLines(const Robot &rb)
    {
        vector<string> lines;

        lines.push_back("Steps: " + to_string(rb.steps));
        lines.push_back(
            "Energy: " + to_string(rb.energy) + "/" + to_string(rb.maxEnergy)
        );
        lines.push_back("Used: " + to_string(rb.totalEnergyUsed));
        lines.push_back("Returns: " + to_string(rb.returnCount));
        lines.push_back("Recharges: " + to_string(rb.rechargeCount));
        lines.push_back("State: " + hudState);

        if (!hudEvents.empty())
        {
            lines.push_back("----------------");
            lines.push_back("Behavior log:");

            int start = max(0, (int)hudEvents.size() - MAX_VISIBLE_HUD_EVENTS);

            for (int i = start; i < (int)hudEvents.size(); i++)
                lines.push_back(trimEventForHUD(hudEvents[i]));
        }

        return lines;
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

void pushHUDEvent(const string &event)
{
    if (event.empty())
        return;

    if (!hudEvents.empty() && hudEvents.back() == event)
        return;

    hudEvents.push_back(event);

    while ((int)hudEvents.size() > MAX_STORED_HUD_EVENTS)
        hudEvents.pop_front();
}

void clearHUDEvents()
{
    hudEvents.clear();
}

void paintHUD(Mat &canvas, const Robot &rb, int delay)
{
    vector<string> lines = buildHUDLines(rb);

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

        if (lines[i] == "Behavior log:")
            color = Scalar(80, 80, 80);

        if (i < 6 && lines[i].rfind("State:", 0) == 0)
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
