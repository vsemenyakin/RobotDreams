#include <iostream>
#include <fstream>
#include <vector>

//#define DebugPrint
#ifdef DebugPrint
#define PrintField(M_FieldName)\
	std::cout << #M_FieldName ": " << M_FieldName << std::endl;
#endif

// =========================

struct Frame
{
    static Frame fromFile(std::ifstream& inFile);

#ifdef DebugPrint
    void print() const;
#endif

    long timestamp_ms;
    long fl_ticks;
    long fr_ticks;
    long bl_ticks;
    long br_ticks;
};

Frame Frame::fromFile(std::ifstream &inFile)
{
    Frame frame;

    inFile >> frame.timestamp_ms
            >> frame.fl_ticks
            >> frame.fr_ticks
            >> frame.bl_ticks
            >> frame.br_ticks;

    return frame;
}

#ifdef DebugPrint
void Frame::print() const
{
    std::cout << "--- Frame ---" << std::endl;

    PrintField(timestamp_ms);
    PrintField(fl_ticks);
    PrintField(fr_ticks);
    PrintField(bl_ticks);
    PrintField(br_ticks);
}
#endif //DebugPrint

// -------------------------

struct DroneState
{
#ifdef DebugPrint
    void print() const;
#endif

    double x;
    double y;
    double theta;
};

#ifdef DebugPrint
void DroneState::print() const
{
    std::cout << "--- DroneState ---" << std::endl;

    PrintField(x);
    PrintField(y);
    PrintField(theta);
}
#endif //DebugPrint

// -------------------------

constexpr long ticks_per_revolution = 1024;
constexpr double wheel_radius_m = 0.3f;
constexpr double wheelbase_m = 1.0f;

// =========================

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }

    const char* inputPath = argv[1];

    std::ifstream file(inputPath);
    if (!file.is_open()) {
        std::cerr << "failed to open file: " << inputPath << "\n";
        return 1;
    }

    std::vector<Frame> frames;
    while (!file.eof()) {
        const auto frame = Frame::fromFile(file);
        frames.push_back(frame);
    }

#ifdef DebugPrint
    for (const Frame& frame : frames) {
        frame.print();
    }
#endif //DebugPrint



    return 0;
}
