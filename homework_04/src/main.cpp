#include <iostream>
#include <fstream>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

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

void writeDroneStateToFile(std::ofstream& outFile, const long inTimeStamp, const DroneState& inDroneState)
{
    outFile << inTimeStamp << " " <<
        inDroneState.x << " " <<
        inDroneState.y << " " <<
        inDroneState.theta << std::endl;
}

// -------------------------

constexpr long ticks_per_revolution = 1024;
constexpr double wheel_radius_m = 0.3f;
constexpr double wheelbase_m = 1.0f;

// =========================

int main(int argc, char** argv)
{
    //--------- Input ---------

    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }

    const char* inputPath = argv[1];

    std::ifstream inputFile(inputPath);
    if (!inputFile.is_open()) {
        std::cerr << "failed to open file: " << inputPath << "\n";
        return 1;
    }

    std::vector<Frame> frames;
    while (!inputFile.eof()) {
        const auto frame = Frame::fromFile(inputFile);
        frames.push_back(frame);
    }

#ifdef DebugPrint
    for (const Frame& frame : frames) {
        frame.print();
    }
#endif //DebugPrint

    //---------- Main logic ---------

    DroneState droneState
    {
        .x = 0,
        .y = 0,
        .theta = 0
    };
    
    std::ofstream outputFile("output.txt");

    writeDroneStateToFile(outputFile, frames[0].timestamp_ms, droneState);
#ifdef DebugPrint
    std::cout << " ========== Simulation ==========" << std::endl;
    droneState.print();
#endif //DebugPrint

    for (size_t i = 1; i < frames.size(); ++i)
    {
        const long d_fl = frames[i].fl_ticks - frames[i - 1].fl_ticks;
        const long d_fr = frames[i].fr_ticks - frames[i - 1].fr_ticks;
        const long d_bl = frames[i].bl_ticks - frames[i - 1].bl_ticks;
        const long d_br = frames[i].br_ticks - frames[i - 1].br_ticks;

        const long d_left  = (d_fl + d_bl) / 2;
        const long d_right = (d_fr + d_br) / 2;

        const double distance_per_tick = 2 * M_PI * wheel_radius_m / ticks_per_revolution;
        const double dL = d_left  * distance_per_tick;
        const double dR = d_right * distance_per_tick;

        const double d = (dL + dR) / 2;
        const double dtheta = (dR - dL) / wheelbase_m;

        droneState.x += d * cos(droneState.theta + dtheta / 2);
        droneState.y += d * sin(droneState.theta + dtheta / 2);
        droneState.theta += dtheta;

        writeDroneStateToFile(outputFile, frames[i].timestamp_ms, droneState);
#ifdef DebugPrint
        droneState.print();
#endif
    }

    outputFile.close();

    return 0;
}
