#include <FL/Fl.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Float_Input.H>
#include <FL/Fl_Int_Input.H>
#include <FL/Fl_Multiline_Output.H>
#include <FL/Fl_Widget.H>
#include <FL/fl_draw.H>

#include <algorithm>
#include <ctime>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "AStarPlanner.h"
#include "DijkstraPlanner.h"
#include "Map.h"
#include "PlannerConfig.h"

namespace {

extern bool g_show_explored;

struct PathLayer {
    std::string name;
    Fl_Color color;
    Fl_Color explored_color;
    std::vector<Point> points;
    std::vector<Point> explored;
};

class MapCanvas : public Fl_Widget {
public:
    MapCanvas(int x, int y, int w, int h) : Fl_Widget(x, y, w, h) {}

    void setMap(const Map &value) {
        map = value;
        layers.clear();
        redraw();
    }

    const Map &getMap() const {
        return map;
    }

    void setEndpoints(const Point &start, const Point &target) {
        s = start;
        t = target;
        redraw();
    }

    void setLayers(std::vector<PathLayer> value) {
        layers = std::move(value);
        redraw();
    }

    void clearLayers() {
        layers.clear();
        redraw();
    }

    void draw() override {
        fl_push_clip(x(), y(), w(), h());
        fl_color(FL_WHITE);
        fl_rectf(x(), y(), w(), h());

        if (map.width() == 0 || map.height() == 0) {
            fl_color(FL_DARK3);
            fl_draw("Load a map to start", x() + 20, y() + 30);
            fl_pop_clip();
            return;
        }

        const int cols = std::max(1, map.width());
        const int rows = std::max(1, map.height());
        const int cell = std::max(4, std::min(w() / cols, h() / rows));
        const int gridW = cell * cols;
        const int gridH = cell * rows;
        const int offsetX = x() + (w() - gridW) / 2;
        const int offsetY = y() + (h() - gridH) / 2;

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                const int px = offsetX + col * cell;
                const int py = offsetY + row * cell;
                if (!map.isFree(col, row)) {
                    fl_color(fl_rgb_color(45, 45, 45));
                } else if (map.hasWeightedCosts()) {
                    const float minCost = map.minTraversableCost();
                    const float currentCost = map.cost(col, row);
                    const int shade = std::clamp(240 - static_cast<int>((currentCost - minCost) * 18.0f), 80, 240);
                    fl_color(fl_rgb_color(shade, shade, 255));
                } else {
                    fl_color(FL_WHITE);
                }
                fl_rectf(px, py, cell, cell);
                fl_color(fl_rgb_color(220, 220, 220));
                fl_rect(px, py, cell, cell);
            }
        }

        for (const auto &layer : layers) {
            if (g_show_explored) {
                for (const auto &explored : layer.explored) {
                    const int px = offsetX + explored.x * cell;
                    const int py = offsetY + explored.y * cell;
                    fl_color(layer.explored_color);
                    fl_line_style(FL_SOLID, 1);
                    fl_rect(px + 2, py + 2, std::max(1, cell - 4), std::max(1, cell - 4));
                    if (cell >= 10) {
                        fl_point(px + cell / 2, py + cell / 2);
                    }
                }
            }

            if (!layer.points.empty()) {
                fl_color(layer.color);
                fl_line_style(FL_SOLID, 3);
                for (size_t i = 1; i < layer.points.size(); ++i) {
                    const int x1 = offsetX + layer.points[i - 1].x * cell + cell / 2;
                    const int y1 = offsetY + layer.points[i - 1].y * cell + cell / 2;
                    const int x2 = offsetX + layer.points[i].x * cell + cell / 2;
                    const int y2 = offsetY + layer.points[i].y * cell + cell / 2;
                    fl_line(x1, y1, x2, y2);
                }
            }
        }
        fl_line_style(0);

        const auto draw_marker = [&](const Point &p, Fl_Color color) {
            const int px = offsetX + p.x * cell;
            const int py = offsetY + p.y * cell;
            fl_color(color);
            fl_rectf(px + 2, py + 2, std::max(1, cell - 3), std::max(1, cell - 3));
        };

        draw_marker(s, FL_GREEN);
        draw_marker(t, FL_RED);

        fl_pop_clip();
    }

private:
    Map map;
    Point s{0, 0};
    Point t{0, 0};
    std::vector<PathLayer> layers;
};

MapCanvas *g_canvas = nullptr;
Fl_Int_Input *g_start_x = nullptr;
Fl_Int_Input *g_start_y = nullptr;
Fl_Int_Input *g_target_x = nullptr;
Fl_Int_Input *g_target_y = nullptr;
Fl_Float_Input *g_turn_penalty_input = nullptr;
Fl_Multiline_Output *g_stats = nullptr;
bool g_show_explored = true;

std::string defaultMapPath() {
    return "data\\example_map.txt";
}

std::string randomMapPath() {
    return "data\\generated_random_map.txt";
}

void setStats(const std::string &text) {
    if (g_stats) {
        g_stats->value(text.c_str());
    }
}

bool readIntInput(Fl_Int_Input *input, int &value) {
    if (!input) {
        return false;
    }
    try {
        value = std::stoi(input->value());
        return true;
    } catch (...) {
        return false;
    }
}

bool readFloatInput(Fl_Float_Input *input, float &value) {
    if (!input) {
        return false;
    }
    try {
        value = std::stof(input->value());
        return true;
    } catch (...) {
        return false;
    }
}

bool getEndpoints(Point &start, Point &target) {
    return readIntInput(g_start_x, start.x) &&
           readIntInput(g_start_y, start.y) &&
           readIntInput(g_target_x, target.x) &&
           readIntInput(g_target_y, target.y);
}

void syncEndpointInputs(const Point &start, const Point &target) {
    std::string sx = std::to_string(start.x);
    std::string sy = std::to_string(start.y);
    std::string tx = std::to_string(target.x);
    std::string ty = std::to_string(target.y);
    if (g_start_x) g_start_x->value(sx.c_str());
    if (g_start_y) g_start_y->value(sy.c_str());
    if (g_target_x) g_target_x->value(tx.c_str());
    if (g_target_y) g_target_y->value(ty.c_str());
}

void syncTurnPenaltyInput(float value) {
    if (g_turn_penalty_input) {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(2);
        oss << value;
        g_turn_penalty_input->value(oss.str().c_str());
    }
}

bool updateTurnPenaltyFromInput(float &value) {
    if (!readFloatInput(g_turn_penalty_input, value)) {
        return false;
    }
    if (value < 0.0f) {
        value = 0.0f;
    }
    setTurnPenalty(value);
    return true;
}

bool writeRandomMapToFile(int width, int height, double obstacleRate, unsigned seed, const std::string &path) {
    if (width <= 0 || height <= 0) {
        return false;
    }

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::uniform_int_distribution<int> costDist(2, 9);
    std::vector<int> grid(width * height, 0);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            grid[y * width + x] = (dist(rng) < obstacleRate) ? 0 : costDist(rng);
        }
    }

    grid[0] = 2;
    grid[(height - 1) * width + (width - 1)] = 2;

    std::ofstream out(path);
    if (!out) {
        return false;
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            out << grid[y * width + x];
            if (x + 1 < width) {
                out << ' ';
            }
        }
        out << '\n';
    }

    return true;
}

void loadMapIntoCanvas(const std::string &path) {
    if (!g_canvas) {
        return;
    }

    Map map;
    if (!map.loadFromTxt(path)) {
        fl_message("Failed to load map: %s", path.c_str());
        return;
    }

    const Point start{0, 0};
    const Point target{map.width() - 1, map.height() - 1};
    g_canvas->setMap(map);
    g_canvas->setEndpoints(start, target);
    g_canvas->clearLayers();
    syncEndpointInputs(start, target);

    std::ostringstream oss;
    oss << "Loaded: " << path << "\n"
        << "Map size: " << map.width() << " x " << map.height() << "\n"
        << "Mode: " << (map.hasWeightedCosts() ? "weighted" : "binary") << "\n"
        << "Min traversable cost: " << map.minTraversableCost();
    setStats(oss.str());
}

template <typename PlannerT>
bool runPlanner(const char *name, Fl_Color color, Fl_Color exploredColor, std::vector<PathLayer> &layers) {
    if (!g_canvas) {
        return false;
    }

    float currentTurnPenalty = turnPenalty();
    updateTurnPenaltyFromInput(currentTurnPenalty);

    const Map &map = g_canvas->getMap();
    if (map.width() == 0 || map.height() == 0) {
        fl_message("Please load a map first.");
        return false;
    }

    Point start;
    Point target;
    if (!getEndpoints(start, target)) {
        fl_message("Start/target inputs must be integers.");
        return false;
    }

    if (start.x < 0 || start.x >= map.width() || start.y < 0 || start.y >= map.height() ||
        target.x < 0 || target.x >= map.width() || target.y < 0 || target.y >= map.height()) {
        fl_message("Start/target are outside the map bounds.");
        return false;
    }

    PlannerT planner;
    const bool ok = planner.plan(map, start, target);
    const auto path = planner.getPath();
    const auto stats = planner.getStats();
    layers.push_back({name, color, exploredColor, path, stats.explored_cells});
    g_canvas->setEndpoints(start, target);
    g_canvas->setLayers(layers);

    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << name << "\n"
        << "success: " << (ok ? "true" : "false") << "\n"
        << "time_ms: " << planner.getStats().time_ms << "\n"
        << "nodes_expanded: " << planner.getStats().nodes_expanded << "\n"
        << "path_cost: " << planner.getStats().path_cost << "\n"
        << "explored_cells: " << stats.explored_cells.size() << "\n"
        << "turn_penalty: " << currentTurnPenalty;
    setStats(oss.str());
    return ok;
}

void load_cb(Fl_Widget *, void *) {
    const char *file = fl_file_chooser("Select map file", "*.txt", defaultMapPath().c_str());
    if (file) {
        loadMapIntoCanvas(file);
    }
}

void run_dijkstra_cb(Fl_Widget *, void *) {
    std::vector<PathLayer> layers;
    runPlanner<DijkstraPlanner>("Dijkstra", FL_BLUE, fl_rgb_color(180, 210, 255), layers);
}

void run_astar_cb(Fl_Widget *, void *) {
    std::vector<PathLayer> layers;
    runPlanner<AStarPlanner>("A*", fl_rgb_color(255, 165, 0), fl_rgb_color(255, 225, 160), layers);
}

void run_both_cb(Fl_Widget *, void *) {
    std::vector<PathLayer> layers;
    const bool dijkstra_ok = runPlanner<DijkstraPlanner>("Dijkstra", FL_BLUE, fl_rgb_color(180, 210, 255), layers);
    const bool astar_ok = runPlanner<AStarPlanner>("A*", fl_rgb_color(255, 165, 0), fl_rgb_color(255, 225, 160), layers);

    std::ostringstream oss;
    oss << "Comparison finished\n"
        << "Dijkstra: " << (dijkstra_ok ? "success" : "failed") << "\n"
        << "A*: " << (astar_ok ? "success" : "failed");
    setStats(oss.str());
}

void clear_cb(Fl_Widget *, void *) {
    if (!g_canvas) {
        return;
    }
    g_canvas->clearLayers();
    setStats("Paths cleared.");
}

void load_default_map() {
    loadMapIntoCanvas(defaultMapPath());
}

void random_map_cb(Fl_Widget *, void *) {
    int width = 40;
    int height = 40;

    if (g_canvas) {
        const Map &current = g_canvas->getMap();
        if (current.width() > 0 && current.height() > 0) {
            width = current.width();
            height = current.height();
        }
    }

    const unsigned seed = static_cast<unsigned>(std::time(nullptr));
    const std::string path = randomMapPath();
    if (!writeRandomMapToFile(width, height, 0.25, seed, path)) {
        fl_message("Failed to generate random map.");
        return;
    }

    loadMapIntoCanvas(path);

    std::ostringstream oss;
    oss << "Random map generated\n"
        << "size: " << width << " x " << height << "\n"
        << "seed: " << seed << "\n"
        << "file: " << path << "\n"
        << "mode: weighted";
    setStats(oss.str());
}

void show_explored_cb(Fl_Widget *widget, void *) {
    auto *toggle = static_cast<Fl_Check_Button *>(widget);
    g_show_explored = toggle->value() != 0;
    if (g_canvas) {
        g_canvas->redraw();
    }
}

} // namespace

int main(int argc, char **argv) {
    Fl_Double_Window win(1220, 820, "Path Planner FLTK Visualizer");

    Fl_Group toolbar(15, 15, 1190, 150);
    Fl_Button load_btn(15, 15, 110, 32, "Load Map");
    Fl_Button dijkstra_btn(135, 15, 130, 32, "Run Dijkstra");
    Fl_Button astar_btn(275, 15, 110, 32, "Run A*");
    Fl_Button both_btn(395, 15, 110, 32, "Run Both");
    Fl_Button clear_btn(515, 15, 110, 32, "Clear Paths");
    Fl_Button random_btn(635, 15, 150, 32, "Random Map");
    Fl_Float_Input turn_penalty(910, 15, 70, 28, "Turn");
    Fl_Check_Button show_explored_btn(995, 15, 170, 28, "Show Explored");

    Fl_Int_Input start_x(15, 65, 70, 28, "Start X");
    Fl_Int_Input start_y(100, 65, 70, 28, "Start Y");
    Fl_Int_Input target_x(215, 65, 70, 28, "Target X");
    Fl_Int_Input target_y(300, 65, 70, 28, "Target Y");

    Fl_Multiline_Output stats(395, 60, 300, 85, "Stats");
    stats.value("Load a map to begin.");

    Fl_Box hint(720, 55, 460, 90,
                "Green = start, red = target, blue/orange = paths\n"
                "Pale cells show explored nodes, blue-tinted cells mean low cost.");
    hint.align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE | FL_ALIGN_TOP_LEFT);

    toolbar.end();

    g_canvas = new MapCanvas(15, 180, 1190, 625);
    g_start_x = &start_x;
    g_start_y = &start_y;
    g_target_x = &target_x;
    g_target_y = &target_y;
    g_turn_penalty_input = &turn_penalty;
    g_stats = &stats;

    syncTurnPenaltyInput(turnPenalty());

    load_btn.callback(load_cb);
    dijkstra_btn.callback(run_dijkstra_cb);
    astar_btn.callback(run_astar_cb);
    both_btn.callback(run_both_cb);
    clear_btn.callback(clear_cb);
    random_btn.callback(random_map_cb);
    show_explored_btn.value(1);
    show_explored_btn.callback(show_explored_cb);

    win.end();
    win.resizable(g_canvas);
    win.show(argc, argv);

    if (argc >= 2) {
        loadMapIntoCanvas(argv[1]);
    } else {
        load_default_map();
    }

    return Fl::run();
}
