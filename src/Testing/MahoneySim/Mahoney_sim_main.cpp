//created by Divyansh Srivastav on 1/13/2026
//this is the main file to test the mahoney filter in a simulation environment

#include "Filters/Mahoney.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include "Math/Quaternion.h"
#include "Math/Vector.h"
#include "Math/Matrix.h"




static constexpr double RAD2DEG = 180.0 / M_PI;

using namespace astra;

int main()
{
    const std::string inPath  = R"(C:\Users\Sriva\Downloads\NAV_data.bin)";
    const std::string outPath = R"(C:\Users\Sriva\Downloads\mahony_out.csv)";

    std::ifstream fin(inPath);
    if (!fin.is_open()) {
        std::cerr << "Failed to open input file: " << inPath << "\n";
        return 1;
    }

    std::ofstream fout(outPath);
    if (!fout.is_open()) {
        std::cerr << "Failed to open output file: " << outPath << "\n";
        return 1;
    }

    // CSV header
    fout << "t,"
         << "roll_gt_deg,pitch_gt_deg,yaw_gt_deg,"
         << "roll_est_deg,pitch_est_deg,yaw_est_deg,"
         << "qw,qx,qy,qz\n";

    // Create your filter
    MahonyAHRS mahony(0.1, 0.0005);

    // ------------------------------
    // Force "no mag calibration"
    // ------------------------------
    mahony._magCalibrated = false; // calibrateMag() will return raw

    // (Optional safety) set identity/zero anyway:
    mahony.hard_iron = astra::Vector<3>(0.0, 0.0, 0.0);

    // soft_iron exists and has allocated memory; set it to identity
    mahony.soft_iron(0,0) = 1.0; mahony.soft_iron(0,1) = 0.0; mahony.soft_iron(0,2) = 0.0;
    mahony.soft_iron(1,0) = 0.0; mahony.soft_iron(1,1) = 1.0; mahony.soft_iron(1,2) = 0.0;
    mahony.soft_iron(2,0) = 0.0; mahony.soft_iron(2,1) = 0.0; mahony.soft_iron(2,2) = 1.0;

    // ------------------------------
    // Force initialized state for dataset replay
    // ------------------------------
    mahony._q = astra::Quaternion(1.0, 0.0, 0.0, 0.0);
    mahony._q.normalize();
    mahony._q0 = mahony._q;
    mahony._biasX = mahony._biasY = mahony._biasZ = 0.0;
    mahony._initialized = true;

    // Dataset example uses dt=0.02 (50Hz). Start with this.
    const double dt = 0.02;
    double t = 0.0;

    std::string line;
    size_t row = 0;

    // Loop until file runs out of data (EOF)
    while (std::getline(fin, line)) {
        if (line.empty()) continue;

        std::istringstream iss(line);
        double v[12];

        for (int i = 0; i < 12; i++) {
            if (!(iss >> v[i])) {
                std::cerr << "Parse error on row " << row << " (expected 12 values)\n";
                return 1;
            }
        }

        // Inputs: acc, gyro, mag
        astra::Vector<3> acc(v[0], v[1], v[2]);
        astra::Vector<3> gyr(v[3], v[4], v[5]);
        astra::Vector<3> mag(v[6], v[7], v[8]);

        // Ground truth angles from file (likely radians -> degrees)
        double roll_gt  = v[9]  * RAD2DEG;
        double pitch_gt = v[10] * RAD2DEG;
        double yaw_gt   = v[11] * RAD2DEG;

        // Dataset note: if yaw truth needs offset correction, enable this:
        // yaw_gt -= 8.3;

        // VERY COMMON GOTCHA:
        // If gyro in dataset is deg/s but your filter expects rad/s, uncomment:
        // gyr = gyr * (M_PI / 180.0);

        // Run your filter
        mahony.update(acc, gyr, mag, dt);

        // Your Quaternion::toEuler321() returns:
        // x=yaw, y=pitch, z=roll  (radians)
        astra::Vector<3> ypr = mahony._q.toEuler321();
        double yaw_est   = ypr.x() * RAD2DEG;
        double pitch_est = ypr.y() * RAD2DEG;
        double roll_est  = ypr.z() * RAD2DEG;

        // Write to CSV
        fout << t << ","
             << roll_gt << "," << pitch_gt << "," << yaw_gt << ","
             << roll_est << "," << pitch_est << "," << yaw_est << ","
             << mahony._q.w() << "," << mahony._q.x() << "," << mahony._q.y() << "," << mahony._q.z()
             << "\n";

        t += dt;
        row++;
    }

    std::cout << "Done. Wrote " << row << " rows to:\n" << outPath << "\n";
    return 0;
}