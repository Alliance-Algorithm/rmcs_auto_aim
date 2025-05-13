#pragma once

#include "opencv2/core/utils/trace.hpp"
#include "opencv2/imgproc.hpp"
#include <numeric>
#include <stack>

enum { XY_SHIFT = 16, XY_ONE = 1 << XY_SHIFT, DRAWING_STORAGE_BLOCK = (1 << 12) - 256 };
struct PolyEdge {
    PolyEdge()
        : y0(0)
        , y1(0)
        , x(0)
        , dx(0)
        , next(0) {}
    
    int y0, y1;
    int64 x, dx;
    PolyEdge* next;
};

static inline void
    CollectPolyEdges(cv::Mat& img, const cv::Point2l* v, int count, std::vector<PolyEdge>& edges) {
    int i, delta = 0;
    cv::Point2l pt0 = v[count - 1], pt1;
    pt0.x           = (pt0.x) << XY_SHIFT;
    pt0.y           = pt0.y + delta;

    edges.reserve(edges.size() + count);

    for (i = 0; i < count; i++, pt0 = pt1) {
        cv::Point2l t0, t1;
        PolyEdge edge;

        pt1   = v[i];
        pt1.x = (pt1.x) << XY_SHIFT;
        pt1.y = (pt1.y + delta);

        cv::Point2l pt0c(pt0), pt1c(pt1);

        t0.y = pt0.y;
        t1.y = pt1.y;
        t0.x = (pt0.x + (XY_ONE >> 1)) >> XY_SHIFT;
        t1.x = (pt1.x + (XY_ONE >> 1)) >> XY_SHIFT;

        if ((unsigned)t0.x >= (unsigned)(img.cols) || (unsigned)t1.x >= (unsigned)(img.cols)
            || (unsigned)t0.y >= (unsigned)(img.rows) || (unsigned)t1.y >= (unsigned)(img.rows)) {
            clipLine(img.size(), t0, t1);

            if (t0.y != t1.y) {
                pt0c.y = t0.y;
                pt1c.y = t1.y;
            }
        }

        pt0c.x = (int64)(t0.x) << XY_SHIFT;
        pt1c.x = (int64)(t1.x) << XY_SHIFT;

        if (pt0.y == pt1.y)
            continue;

        edge.dx = (pt1c.x - pt0c.x) / (pt1c.y - pt0c.y);
        if (pt0.y < pt1.y) {
            edge.y0 = (int)(pt0.y);
            edge.y1 = (int)(pt1.y);
            edge.x =
                pt0c.x + (pt0.y - pt0c.y) * edge.dx;
        } else {
            edge.y0 = (int)(pt1.y);
            edge.y1 = (int)(pt0.y);
            edge.x =
                pt1c.x + (pt1.y - pt1c.y) * edge.dx; 
        }
        edges.push_back(edge);
    }
}

static inline std::vector<cv::Point>
    FillEdgeCollection(cv::Mat& img, std::vector<PolyEdge>& edges) {
    PolyEdge tmp;
    int i, y, total = (int)edges.size();
    cv::Size size = img.size();
    PolyEdge* e;
    int y_max = std::numeric_limits<int>::min(), y_min = std::numeric_limits<int>::max();
    int64 x_max = 0xFFFFFFFFFFFFFFFF, x_min = 0x7FFFFFFFFFFFFFFF;
    int delta    = XY_ONE - 1;

    if (total < 2)
        return {};

    for (i = 0; i < total; i++) {
        PolyEdge& e1 = edges[i];
        CV_Assert(e1.y0 < e1.y1);
        int64 x1 = e1.x + (e1.y1 - e1.y0) * e1.dx;
        y_min    = std::min(y_min, e1.y0);
        y_max    = std::max(y_max, e1.y1);
        x_min    = std::min(x_min, e1.x);
        x_max    = std::max(x_max, e1.x);
        x_min    = std::min(x_min, x1);
        x_max    = std::max(x_max, x1);
    }

    if (y_max < 0 || y_min >= size.height || x_max < 0 || x_min >= ((int64)size.width << XY_SHIFT))
        return {};

    std::sort(edges.begin(), edges.end(), [](const PolyEdge& e1, const PolyEdge& e2) {
        return e1.y0 - e2.y0 ? e1.y0 < e2.y0 : e1.x - e2.x ? e1.x < e2.x : e1.dx < e2.dx;
    });

    tmp.y0 = std::numeric_limits<int>::max();
    edges.push_back(tmp); 
    i        = 0;
    tmp.next = nullptr;
    e        = &edges[i];
    y_max    = std::min(y_max, size.height);

    std::vector<cv::Point> points;
    for (y = e->y0; y < y_max; y++) {
        PolyEdge *last, *prelast, *keep_prelast;
        int draw     = 0;
        int clipline = y < 0;

        prelast = &tmp;
        last    = tmp.next;
        while (last || e->y0 == y) {
            if (last && last->y1 == y) {
                prelast->next = last->next;
                last          = last->next;
                continue;
            }
            keep_prelast = prelast;
            if (last && (e->y0 > y || last->x < e->x)) {
                prelast = last;
                last    = last->next;
            } else if (i < total) {
                prelast->next = e;
                e->next       = last;
                prelast       = e;
                e             = &edges[++i];
            } else
                break;

            if (draw) {
                if (!clipline) {
                    int x1, x2;

                    if (keep_prelast->x > prelast->x) {
                        x1 = (int)((prelast->x + delta) >> XY_SHIFT);
                        x2 = (int)(keep_prelast->x >> XY_SHIFT);
                    } else {
                        x1 = (int)((keep_prelast->x + delta) >> XY_SHIFT);
                        x2 = (int)(prelast->x >> XY_SHIFT);
                    }

                    if (x1 < size.width && x2 >= 0) {
                        if (x1 < 0)
                            x1 = 0;
                        if (x2 >= size.width)
                            x2 = size.width - 1;
                        for (int i = x1; i <= x2; i++)
                            points.emplace_back(i, y);
                    }
                }
                keep_prelast->x += keep_prelast->dx;
                prelast->x += prelast->dx;
            }
            draw ^= 1;
        }

        keep_prelast = 0;

        do {
            prelast                 = &tmp;
            last                    = tmp.next;
            PolyEdge* last_exchange = 0;

            while (last != keep_prelast && last->next != 0) {
                PolyEdge* te = last->next;

                if (last->x > te->x) {
                    prelast->next = te;
                    last->next    = te->next;
                    te->next      = last;
                    prelast       = te;
                    last_exchange = prelast;
                } else {
                    prelast = last;
                    last    = te;
                }
            }
            if (last_exchange == nullptr)
                break;
            keep_prelast = last_exchange;
        } while (keep_prelast != tmp.next && keep_prelast != &tmp);
    }

    return points;
}

static inline std::vector<std::vector<cv::Point>>
    fillContours(cv::InputOutputArray img, cv::InputArrayOfArrays pts) {
    CV_TRACE_FUNCTION();

    const bool manyContours = pts.kind() == cv::_InputArray::STD_VECTOR_VECTOR
                           || pts.kind() == cv::_InputArray::STD_VECTOR_MAT;
    int i, ncontours = manyContours ? (int)pts.total() : 1;
    if (ncontours == 0)
        return {};

    cv::Mat img_mat = img.getMat();
    std::vector<std::vector<cv::Point>> contour_points;
    for (i = 0; i < ncontours; i++) {
        cv::Mat p = pts.getMat(manyContours ? i : -1);
        CV_Assert(p.checkVector(2, CV_32S) > 0);
        const auto ptsptr= p.ptr<cv::Point>();
        const auto npts   = p.rows * p.cols * p.channels() / 2;
        if (npts > 0 && ptsptr) {
            std::vector<cv::Point2l> _pts(ptsptr, ptsptr + npts);
            std::vector<PolyEdge> edges;
            edges.reserve(npts+ 1);
            CollectPolyEdges(img_mat, _pts.data(), npts, edges);
            contour_points.emplace_back(FillEdgeCollection(img_mat, edges));
        }
    }

    return contour_points;
}

static inline std::vector<std::vector<cv::Point>> get_contours_points(
    cv::InputOutputArray _image, cv::InputArrayOfArrays _contours, int contourIdx,
    cv::InputArray _hierarchy=-1, int maxLevel=0) {
    CV_TRACE_FUNCTION();

    const size_t ncontours = _contours.total();
    if (ncontours==0)
        return {};
    CV_Assert(ncontours <= (size_t)std::numeric_limits<int>::max());

    cv::Mat image                 = _image.getMat();
    cv::Mat_<cv::Vec4i> hierarchy = _hierarchy.getMat();

    int i = 0, end = (int)ncontours;
    if (contourIdx >= 0) {
        i   = contourIdx;
        end = i + 1;
    }
    std::vector<int> indexesToFill;
    if (hierarchy.empty() || maxLevel == 0) {
        indexesToFill.resize(end - i);
        std::iota(indexesToFill.begin(), indexesToFill.end(), i);
    } else {
        std::stack<int> indexes;
        for (; i != end; ++i) {
            if (hierarchy(i)[3] < 0 || contourIdx >= 0)
                indexes.push(i);
        }
        while (!indexes.empty()) {
            const int cur = indexes.top();
            indexes.pop();

            int curLevel = -1;
            int par      = cur;
            while (par >= 0) {
                par = hierarchy(par)[3];   
                ++curLevel;
            }
            if (curLevel <= maxLevel) {
                indexesToFill.push_back(cur);
            }

            int next = hierarchy(cur)[2]; 
            while (next > 0) {
                indexes.push(next);
                next = hierarchy(next)[0]; 
            }
        }
    }
    std::vector<cv::Mat> contoursToFill;
    contoursToFill.reserve(indexesToFill.size());
    for (const int& idx : indexesToFill)
        contoursToFill.emplace_back(_contours.getMat(idx));

    return fillContours(image, contoursToFill);
}