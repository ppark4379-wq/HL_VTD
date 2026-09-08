#include "planning/global/mission_corridor.hpp"
#include <cerrno>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#ifndef _WIN32
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif

bool preparePrecomputedMission(MissionCorridorData& output,
                               PathGeneratorData& map,
                               const std::string& checkpoints,
                               const std::string& xodrPath,
                               const std::string& pointNetworkDirectory,
                               const std::string& directory,
                               std::string* error) {
    output = {};
    try {
        namespace fs = std::filesystem;
        auto dest = fs::absolute(directory);
#ifndef _WIN32
        // Separate simultaneous startup/config checks so neither can load
        // the other mission's checkpoint result.
        dest /= "startup-"+std::to_string(getpid());
#endif
        if (!exportMissionMap(map, dest.string()))
            throw std::runtime_error("cannot export loaded XODR lane geometry");
        const auto artifact = dest/"complete_network.json";
        std::vector<std::string> arguments{
            "python3", std::string(HL_VTD_SOURCE_DIR)+"/tools/verification/prepare_runtime_mission.py",
            "--geometry", dest.string(), "--checkpoints", fs::absolute(checkpoints).string(),
            "--output", artifact.string(), "--xodr", fs::absolute(xodrPath).string(),
            "--point-network", fs::absolute(pointNetworkDirectory).string()};
        // Explicitly scoped first-junction experiment requested for M12. The
        // Python adapter also verifies the exact XODR gateway before adding
        // surface-only spans; other scenarios retain identical route data.
        const auto trialCheckpoints = fs::path(HL_VTD_SOURCE_DIR) /
                                      "scenario/mixed/M12/checkpoints.csv";
        if (fs::weakly_canonical(checkpoints) ==
            fs::weakly_canonical(trialCheckpoints)) {
            arguments.insert(arguments.end(),
                             {"--surface-trial", "m12-g1-exit-apron"});
        }
        std::cout << "[mission] precomputing checkpoint/XODR network before VTD connection\n" << std::flush;
#ifndef _WIN32
        // No shell interpolation: spaces, Korean paths and metacharacters in
        // checkpoint filenames are arguments, never commands.
        std::vector<char*> argv;
        for (auto& arg : arguments) argv.push_back(arg.data());
        argv.push_back(nullptr);
        pid_t child{};
        if (posix_spawnp(&child, argv[0], nullptr, nullptr, argv.data(), environ) != 0)
            throw std::runtime_error("cannot start Python3 mission generator (requires numpy)");
        int status{}; pid_t waited;
        do { waited = waitpid(child, &status, 0); } while (waited < 0 && errno == EINTR);
        if (waited < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
            throw std::runtime_error("whole-mission generation failed; VTD driving was not started");
#else
        throw std::runtime_error("headless mission startup currently requires POSIX");
#endif
        if (!loadPrecomputedMission(output, map, artifact.string(), error)) return false;
        std::cout << "[mission] topology authority loaded from " << artifact
                  << "; legacy route.csv is not gateway authority\n";
        return true;
    } catch (const std::exception& ex) {
        if (error) *error = ex.what();
        return false;
    }
}
