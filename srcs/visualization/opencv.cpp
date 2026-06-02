#include "opencv.h"

#include "entity_renderer.h"
#include "hud_renderer.h"
#include "map_renderer.h"
#include "visual_assets.h"
#include "visual_layout.h"

#include <filesystem>
#include <opencv2/opencv.hpp>

using namespace cv;

namespace
{
    Mat lastFrame;
    bool showCoordinateHeaders = false;
    bool testSpeedEnabled = false;

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

    void handleKeyboard(int key)
    {
        if (key < 0)
            return;

        if (key == 'c' || key == 'C')
        {
            showCoordinateHeaders = !showCoordinateHeaders;

            pushHUDEvent(
                showCoordinateHeaders
                    ? "Coordinate headers enabled."
                    : "Coordinate headers disabled."
            );

            return;
        }

        if (isSpaceKey(key))
        {
            testSpeedEnabled = !testSpeedEnabled;

            pushHUDEvent(
                testSpeedEnabled
                    ? "Test speed enabled: 5x."
                    : "Test speed disabled: 1x."
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

    int key = waitKey(delay);
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
    return testSpeedEnabled ? 5 : 1;
}

bool isTestSpeedEnabled()
{
    return testSpeedEnabled;
}
