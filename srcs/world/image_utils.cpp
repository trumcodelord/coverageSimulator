#include <opencv2/opencv.hpp>

using namespace cv;

void overlayImage(Mat &background, const Mat &icon, Point center, int drawSize)
{
    if (icon.empty()) return;
    if (drawSize <= 0) return;
    if (background.empty()) return;

    Mat resized;
    resize(icon, resized, Size(drawSize, drawSize));

    int x0 = center.x - drawSize / 2;
    int y0 = center.y - drawSize / 2;

    bool hasAlpha = (resized.channels() == 4);

    for (int y = 0; y < resized.rows; y++)
    {
        int by = y0 + y;
        if (by < 0 || by >= background.rows) continue;

        for (int x = 0; x < resized.cols; x++)
        {
            int bx = x0 + x;
            if (bx < 0 || bx >= background.cols) continue;

            float alpha = 1.0f;
            Vec3b srcColor;

            if (hasAlpha)
            {
                Vec4b px = resized.at<Vec4b>(y, x);
                srcColor = Vec3b(px[0], px[1], px[2]);
                alpha = px[3] / 255.0f;

                if (alpha <= 1e-6f)
                    continue;
            }
            else
            {
                Vec3b px = resized.at<Vec3b>(y, x);
                srcColor = px;
            }

            Vec3b &bg = background.at<Vec3b>(by, bx);
            for (int c = 0; c < 3; c++)
            {
                bg[c] = (uchar)(bg[c] * (1.0f - alpha) + srcColor[c] * alpha);
            }
        }
    }
}

Mat rotateIconForOverlay(const Mat &src, int drawSize, double angleDeg)
{
    if (src.empty())
        return src;

    if (drawSize <= 0)
        return src;

    Mat resized;
    resize(src, resized, Size(drawSize, drawSize));

    Point2f center((resized.cols - 1) / 2.0f, (resized.rows - 1) / 2.0f);
    Mat rot = getRotationMatrix2D(center, angleDeg, 1.0);

    Mat dst;
    warpAffine(
        resized,
        dst,
        rot,
        resized.size(),
        INTER_LINEAR,
        BORDER_CONSTANT,
        Scalar(0, 0, 0, 0)
    );

    return dst;
}
