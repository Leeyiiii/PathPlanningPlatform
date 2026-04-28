#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <ctime>
#include <string>

int main(int argc, char** argv){
    int w = 50, h = 50;
    unsigned seed = static_cast<unsigned>(std::time(nullptr));
    if (argc>=3){ w = std::stoi(argv[1]); h = std::stoi(argv[2]); }
    if (argc>=4) seed = static_cast<unsigned>(std::stoi(argv[3]));
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> d(0.0,1.0);
    std::vector<int> grid(w*h,0);
    for (int y=0;y<h;++y) for (int x=0;x<w;++x){
        double p = d(rng);
        grid[y*w+x] = (p<0.25) ? 1 : 0; // 25% obstacles
    }
    grid[0]=0; grid[(h-1)*w + (w-1)] = 0;
    std::string out = "data\\random_large.txt";
    std::ofstream f(out);
    for (int y=0;y<h;++y){
        for (int x=0;x<w;++x){
            f << grid[y*w+x];
            if (x+1<w) f << ' ';
        }
        f << '\n';
    }
    std::cout << "Wrote " << out << " ("<<w<<"x"<<h<<") seed="<<seed<<"\n";
    return 0;
}
