#include "unity.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {
const std::vector<std::string> kRequiredPreprocessedCsvFiles = {
    "data/MARTHA_3-8_1.3_B2_SingleID_transformed.csv",
    "data/MARTHA_IREC_2025_B2_transformed.csv",
    "data/AA Data Collection - Second Launch Trimmed.csv",
};

bool fileExists(const std::string& path) {
    std::ifstream file(path.c_str());
    return file.good();
}
}  // namespace

void setUp(void) {}
void tearDown(void) {}

void test_preprocessed_csv_data_exists(void) {
    std::vector<std::string> missingFiles;
    for (const std::string& file : kRequiredPreprocessedCsvFiles) {
        if (!fileExists(file)) {
            missingFiles.push_back(file);
        }
    }

    if (!missingFiles.empty()) {
        std::cerr << "\n";
        std::cerr << "==============================================================\n";
        std::cerr << "MISSING PREPROCESSED CSV TEST DATA\n";
        std::cerr << "The following required files are missing from data/:\n";
        for (const std::string& file : missingFiles) {
            std::cerr << "  - " << file << "\n";
        }
        std::cerr << "Download/add them, then rerun: pio test\n";
        std::cerr << "Instructions: data/README.md\n";
        std::cerr << "==============================================================\n";
        std::cerr << "\n";
        TEST_FAIL_MESSAGE("Preprocessed CSV test data missing. See message above.");
    }
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_preprocessed_csv_data_exists);
    return UNITY_END();
}
