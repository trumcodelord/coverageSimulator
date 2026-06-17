#include "opencv.h"

#include "dynamic_obstacle.h"
#include "entity_renderer.h"
#include "hud_renderer.h"
#include "map_renderer.h"
#include "visual_assets.h"
#include "visual_layout.h"

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <sstream>

using namespace cv;

namespace
{
    Mat lastFrame;
    bool showCoordinateHeaders = false;
    int speedMultiplier = 1;

    void ensureParentDirectory(const std::string &path)
    {
        std::filesystem::path p(path);
        std::filesystem::path parent = p.parent_path();

        if (!parent.empty())
            std::filesystem::create_directories(parent);
    }

    bool isSpaceKey(int key)
    {
        return key == 32;
    }

    bool isManualVehicleToggleKey(int key)
    {
        int low = key & 0xFF;
        return key == '`' || key == '~' || low == '`' || low == '~';
    }

    int manualVehicleDirFromArrowKey(int key)
    {
        int low = key & 0xFF;

        // OpenCV waitKeyEx arrow codes on Windows.
        if (key == 2424832) return 3; // left
        if (key == 2490368) return 2; // up
        if (key == 2555904) return 1; // right
        if (key == 2621440) return 0; // down

        // Some OpenCV builds expose compact arrow codes.
        if (low == 81) return 3; // left
        if (low == 82) return 2; // up
        if (low == 83) return 1; // right
        if (low == 84) return 0; // down

        return -1;
    }

    void cycleSpeed()
    {
        if (speedMultiplier == 1)
            speedMultiplier = 5;
        else if (speedMultiplier == 5)
            speedMultiplier = 10;
        else
            speedMultiplier = 1;
    }

    void pushManualVehicleTargetEvent()
    {
        int idx = manualControlledVehicleIndex();
        Cell p = manualControlledVehicleCell();

        std::ostringstream oss;
        oss << "[DEV] Manual vehicle ON: V#"
            << (idx + 1)
            << " at ("
            << p.r
            << ","
            << p.c
            << ").";

        pushHUDEvent(oss.str());
    }

    void handleKeyboard(int key)
    {
        if (key < 0)
            return;

        if (isManualVehicleToggleKey(key))
        {
            if (!isManualVehicleControlEnabled() &&
                !hasManualControllableVehicle())
            {
                pushHUDEvent("[DEV] Khong co VEHICLE de dieu khien.");
                return;
            }

            bool enabled = toggleManualVehicleControl();

            if (enabled)
                pushManualVehicleTargetEvent();
            else
                pushHUDEvent("[DEV] Manual vehicle OFF.");

            return;
        }

        int manualDir = manualVehicleDirFromArrowKey(key);

        if (manualDir != -1)
        {
            if (!isManualVehicleControlEnabled())
                return;

            bool moved = manualVehicleControlStep(manualDir);

            pushHUDEvent(
                moved
                    ? "[DEV] Manual vehicle moved."
                    : "[DEV] Manual vehicle move blocked."
            );

            return;
        }

        if (key == 'c' || key == 'C')
        {
            showCoordinateHeaders = !showCoordinateHeaders;

            pushHUDEvent(
                showCoordinateHeaders
                    ? "[VIEW] Da bat toa do."
                    : "[VIEW] Da tat toa do."
            );

            return;
        }

        if (isSpaceKey(key))
        {
            cycleSpeed();

            pushHUDEvent(
                speedMultiplier == 1
                    ? "[SPEED] Da ve toc do 1x."
                    : speedMultiplier == 5
                        ? "[SPEED] Da bat toc do 5x."
                        : "[SPEED] Da bat toc do 10x."
            );

            return;
        }
    }
}

void initWindow()
{
    namedWindow(visualWindowName(), WINDOW_NORMAL);
    setWindowProperty(visualWindowName(), WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

    Rect rect = getWindowImageRect(visualWindowName());

    if (rect.width > 0 && rect.height > 0)
        updateVisualScreenSize(rect.width, rect.height);

    setupVisualLayout();
    loadVisualizationAssets();
}

void closeWindow()
{
    destroyWindow(visualWindowName());
}

void drawFrame(const Robot &rb, const CoverageContext &ctx, bool showPath, int delay)
{
    Mat canvas(
        visualCanvasHeight(),
        visualCanvasWidth(),
        CV_8UC3,
        Scalar(240, 240, 240)
    );

    paintMapCells(canvas);
    paintGridLines(canvas);

    if (showCoordinateHeaders)
        paintCoordinateHeaders(canvas);

    paintTrail(canvas, rb);

    if (showPath)
        paintPath(canvas, rb);

    paintBase(canvas, rb);
    paintDynamicObstacles(canvas);
    paintRobot(canvas, rb, ctx);
    paintHUD(canvas, rb, delay);

    lastFrame = canvas.clone();

    imshow(visualWindowName(), canvas);
}

void waitFrame(int delay)
{
    if (delay < 0)
        delay = 0;

    int key = waitKeyEx(delay);
    handleKeyboard(key);
}

bool saveCurrentFrame(const std::string &filename)
{
    if (lastFrame.empty())
        return false;

    ensureParentDirectory(filename);
    return imwrite(filename, lastFrame);
}

int testSpeedMultiplier()
{
    return speedMultiplier;
}

bool isTestSpeedEnabled()
{
    return speedMultiplier > 1;
}
