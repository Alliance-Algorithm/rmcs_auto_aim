#pragma once

#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>

class ScanLine {
public:
    static inline std::vector<cv::Point>
        get_points(const cv::Mat& image, const std::vector<cv::Point>& contour) {
        if (contour.empty())
            return {};

        std::vector<cv::Point2l> points;

        points.reserve(contour.size());
        for (const auto& point : contour)
            points.emplace_back(point);

        auto edges = collect_poly_edges(points);

        return fill_edge_collection(image, edges);
    }

private:
    struct PolyEdge {
        PolyEdge()
            : y0(0)
            , y1(0)
            , x(0)
            , dx(0)
            , next(nullptr) {}

        int y0, y1;
        int64 x, dx;
        PolyEdge* next;
    };

    enum { XY_SHIFT = 16, XY_ONE = 1 << XY_SHIFT, DRAWING_STORAGE_BLOCK = (1 << 12) - 256 };

    static inline std::vector<cv::Point>
        fill_edge_collection(const cv::Mat& img, std::vector<PolyEdge>& edges) {
        PolyEdge tmp;
        int i, y, total = (int)edges.size();
        cv::Size size = img.size();
        PolyEdge* e;
        int y_max = INT_MIN, y_min = std::numeric_limits<int>::max();
        int64 x_max = -1, x_min = 0x7FFFFFFFFFFFFFFF;

        if (total < 2)
            return {};

        for (i = 0; i < total; i++) {
            PolyEdge& e1 = edges[i];
            assert(e1.y0 < e1.y1);
            // Determine x-coordinate of the end of the edge.
            // (This is not necessary x-coordinate of any vertex in the array.)
            int64 x1 = e1.x + (e1.y1 - e1.y0) * e1.dx;
            y_min    = std::min(y_min, e1.y0);
            y_max    = std::max(y_max, e1.y1);
            x_min    = std::min(x_min, e1.x);
            x_max    = std::max(x_max, e1.x);
            x_min    = std::min(x_min, x1);
            x_max    = std::max(x_max, x1);
        }

        if (y_max < 0 || y_min >= size.height || x_max < 0
            || x_min >= ((int64)size.width << XY_SHIFT))
            return {};

        std::sort(edges.begin(), edges.end(), [](const PolyEdge& e1, const PolyEdge& e2) {
            return e1.y0 - e2.y0 ? e1.y0 < e2.y0 : e1.x - e2.x ? e1.x < e2.x : e1.dx < e2.dx;
        });

        tmp.y0 = std::numeric_limits<int>::max();
        edges.push_back(tmp); // after this point we do not add
                              // any elements to edges, thus we can use pointers
        i        = 0;
        tmp.next = nullptr;
        e        = &edges[i];
        y_max    = MIN(y_max, size.height);

        std::vector<cv::Point> points;
        for (y = e->y0; y < y_max; y++) {
            PolyEdge *last, *prelast, *keep_prelast;
            int sort_flag = 0;
            int draw      = 0;
            int clipline  = y < 0;

            prelast = &tmp;
            last    = tmp.next;
            while (last || e->y0 == y) {
                if (last && last->y1 == y) {
                    // exclude edge if y reaches its lower point
                    prelast->next = last->next;
                    last          = last->next;
                    continue;
                }
                keep_prelast = prelast;
                if (last && (e->y0 > y || last->x < e->x)) {
                    // go to the next edge in active list
                    prelast = last;
                    last    = last->next;
                } else if (i < total) {
                    // insert new edge into active list if y reaches its upper point
                    prelast->next = e;
                    e->next       = last;
                    prelast       = e;
                    e             = &edges[++i];
                } else
                    break;

                if (draw) {
                    if (!clipline) {
                        // convert x's from fixed-point to image coordinates
                        int x1, x2;

                        if (keep_prelast->x > prelast->x) {
                            x1 = (int)((prelast->x + XY_ONE - 1) >> XY_SHIFT);
                            x2 = (int)(keep_prelast->x >> XY_SHIFT);
                        } else {
                            x1 = (int)((keep_prelast->x + XY_ONE - 1) >> XY_SHIFT);
                            x2 = (int)(prelast->x >> XY_SHIFT);
                        }

                        // clip and draw the line
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

            // sort edges (using bubble sort)
            keep_prelast = 0;

            do {
                prelast = &tmp;
                last    = tmp.next;

                while (last != keep_prelast && last->next != 0) {
                    PolyEdge* te = last->next;

                    // swap edges
                    if (last->x > te->x) {
                        prelast->next = te;
                        last->next    = te->next;
                        te->next      = last;
                        prelast       = te;
                        sort_flag     = 1;
                    } else {
                        prelast = last;
                        last    = te;
                    }
                }
                keep_prelast = prelast;
            } while (sort_flag && keep_prelast != tmp.next && keep_prelast != &tmp);
        }
        return points;
    }

    static inline std::vector<PolyEdge> collect_poly_edges(const std::vector<cv::Point2l>& points) {
        int delta              = ((1 << 0) >> 1);
        const auto points_size = points.size();
        cv::Point2l pt0        = points[points_size - 1], pt1;
        pt0.x                  = (pt0.x) << XY_SHIFT;
        pt0.y                  = pt0.y + delta;

        std::vector<PolyEdge> edges;
        edges.reserve(edges.size() + points_size);
        for (std::size_t i = 0; i < points_size; i++, pt0 = pt1) {
            cv::Point2l t0, t1;
            pt1   = points[i];
            pt1.x = (pt1.x) << XY_SHIFT;
            pt1.y = pt1.y + delta;

            t0.x = pt0.x;
            t1.x = pt1.x;
            t0.y = pt0.y << XY_SHIFT;
            t1.y = pt1.y << XY_SHIFT;

            if (pt0.y == pt1.y)
                continue;

            PolyEdge edge;
            if (pt0.y < pt1.y) {
                edge.y0 = static_cast<int>(pt0.y);
                edge.y1 = static_cast<int>(pt1.y);
                edge.x  = pt0.x;
            } else {
                edge.y0 = static_cast<int>(pt1.y);
                edge.y1 = static_cast<int>(pt0.y);
                edge.x  = pt1.x;
            }
            edge.dx = (pt1.x - pt0.x) / (pt1.y - pt0.y);
            edges.emplace_back(edge);
        }
        return edges;
    }
};