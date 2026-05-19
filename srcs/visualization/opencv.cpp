#include "opencv.h"

#include "entity_renderer.h"
#include "hud_renderer.h"
#include "map_renderer.h"
#include "visual_assets.h"
#include "visual_layout.h"

#include <opencv2/opencv.hpp>

using namespace cv;

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

void drawFrame(const Robot &rb, bool showPath, int delay)
{
    Mat canvas(
        visualCanvasHeight(),
        visualCanvasWidth(),
        CV_8UC3,
        Scalar(240, 240, 240)
    );

    paintMapCells(canvas);
    paintGridLines(canvas);
    paintTrail(canvas, rb);

    if (showPath)
        paintPath(canvas, rb);

    paintBase(canvas, rb);
    paintDynamicObstacles(canvas);
    paintRobot(canvas, rb);
    paintHUD(canvas, rb, delay);

    imshow(visualWindowName(), canvas);
}

void waitFrame(int delay)
{
    if (delay < 0)
        delay = 0;

    waitKey(delay);
}
