// Part A: This is an extension task that requires you to decode sensor data from CAN log files.
// CAN (Controller Area Network) is a communication standard used in automotive applications (including Redback cars)
// to allow communication between sensors and controllers.
//
// Your Task: Using the signal definitions in SteeringBench.dbc, read each CAN capture in data/
// and turn it into a CSV with one row per decoded frame:
// t,u_commanded,y_measured
// eg:
// 0,15.0,0.0
// 0.005,15.0,0.0
// ...
// where t is the frame timestamp minus the first kept frame's timestamp (s), u_commanded is
// the decoded CmdAngularRate (deg/s), and y_measured is the decoded MeasuredAngle (deg).
// The above values are not real numbers; they are only there to show the expected data output format.
// Do this for all three captures:
// data/step_test.log       ->  data/step_test.csv
// data/reversal_test.log   ->  data/reversal_test.csv
// data/deadband_test.log   ->  data/deadband_test.csv
//
// The Row type, writeCsv(), and main() below are provided -- they loop the three logs, call your
// decodeLog(), and write the CSV in exactly the format above. You just need to implement decodeLog().
//
// You do not need to use any external libraries. Use the resources below to understand how to
// extract sensor data.
// Hint: Think about manual bit masking and shifting, data types required,
// what formats are used to represent values, etc.
// Resources:
// https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial
// https://www.csselectronics.com/pages/can-dbc-file-database-intro
//
// Sanity check: plot your CSVs (python3 plot_data.py) and compare against the pre-plotted
// data/*.png files -- they should match.
//
// Build & run (from the TA/ folder):
//     c++ -std=c++17 Question-A.cc -o decode
//     ./decode

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

// One output row.
struct Row {
    double t;            // seconds since the first kept frame
    double u_commanded;  // deg/s
    double y_measured;   // deg
};

// Read the candump log at `path` and return one Row per STEER_ActuatorLog frame, in order.
// Push one Row{t, u_commanded, y_measured} per kept frame.
std::vector<Row> decodeLog(const std::string& path) {
    std::vector<Row> rows;

    //find a way to get the particular characters you are looking for
    //physical_value = raw_integer * scale + offset
    std::ifstream file(path);
    std::string line;
    bool havebli = false;
    double baselt = 0.0;

    //i notice that some of the data is not from the 512 bit CAN IDs relevant to us, so i assume they need to be filtered out
    while (std::getline(file, line)) {
        size_t hash = line.find('#');
        size_t sp1 = line.find_last_of(' ') + 1;
        std::string id0 = line.substr(sp1, hash - sp1);
        unsigned long id = std::stoul(id0, nullptr, 16);

        std::string mdatahex = line.substr(hash + 1);

        if (id != 512){
            continue;
        }

        std::string bytes[8];
        for (int i = 0; i < 8; ++i) {
            bytes[i] = mdatahex.substr(i * 2, 2);
        }

        std::string cb1 = bytes[3] + bytes[2];
        long rawangr  = std::stoul(cb1, nullptr, 16);

        if (rawangr >= 32768) {
            rawangr -= 65536;
        }

        double u_commanded1 = rawangr * 0.1 + 0;

        std::string cb2 = bytes[1] + bytes[0];
        long rawMA  = std::stoul(cb2, nullptr, 16);

        if (rawMA >= 32768) {
            rawMA -= 65536;
        }

        double y_measured1 = rawMA * 0.1 + 0;

        size_t bkt = line.find(')');
        std::string timeraw = line.substr(1, bkt - 1);
        double time0 = std::stod(timeraw);
        if (!havebli) {
        baselt = time0;
        havebli = true;
        }
        double t1 = time0 - baselt;

        Row r;
        r.t = t1;
        r.u_commanded = u_commanded1;
        r.y_measured = y_measured1;

        rows.push_back(r);
    }

    // TODO: your code here
    // remove once you open the file

    return rows;
}

// Provided -- writes the rows to a CSV in the required format. Do not change.
void writeCsv(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream f(path);
    f << "t,u_commanded,y_measured\n";
    for (const Row& r : rows)
        f << r.t << "," << r.u_commanded << "," << r.y_measured << "\n";
}

// Provided -- runs decodeLog() + writeCsv() for each of the three captures.
int main() {
    const char* names[] = {"step_test", "reversal_test", "deadband_test"};
    for (const char* n : names) {
        const std::string in  = std::string("data/") + n + ".log";
        const std::string out = std::string("data/") + n + ".csv";
        const std::vector<Row> rows = decodeLog(in);
        writeCsv(out, rows);
        std::printf("%-14s %6zu frames -> %s\n", n, rows.size(), out.c_str());
    }
    return 0;
}
