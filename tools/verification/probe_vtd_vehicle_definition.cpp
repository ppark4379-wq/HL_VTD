// Offline forensic probe for ONE known VTD 2025.2.438 x86-64 plugin build.
// Instantiates a private plugin object; never connects to a running VTD instance.
// Internal offsets were verified from this exact binary, not from a public ABI.
// Refuse any other binary before dlopen or reading internal fields.
#include <dlfcn.h>
#include <openssl/evp.h>

#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
constexpr char kKnownSha256[] =
    "5889bdd536f67589fd4a038973c2d1ed956494790c31492643d100b592a7d0ed";
constexpr std::size_t kModelOffset = 0x380;
constexpr std::size_t kWheelbaseOffset = 0xc8;

std::string sha256(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot open plugin: " + path);
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
        EVP_MD_CTX_new(), EVP_MD_CTX_free);
    if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
        throw std::runtime_error("Cannot initialize SHA-256");
    std::array<char, 65536> buffer{};
    while (input) {
        input.read(buffer.data(), buffer.size());
        if (EVP_DigestUpdate(context.get(), buffer.data(), input.gcount()) != 1)
            throw std::runtime_error("Cannot hash plugin");
    }
    if (!input.eof()) throw std::runtime_error("Plugin read failed");
    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    unsigned int size = 0;
    if (EVP_DigestFinal_ex(context.get(), digest.data(), &size) != 1)
        throw std::runtime_error("Cannot finalize SHA-256");
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (unsigned int i = 0; i < size; ++i)
        output << std::setw(2) << static_cast<unsigned int>(digest[i]);
    return output.str();
}

template <class Function> Function symbol(void* library, const char* name) {
    dlerror();
    void* address = dlsym(library, name);
    if (const char* error = dlerror()) throw std::runtime_error(error);
    if (!address) throw std::runtime_error("Null symbol: " + std::string(name));
    return reinterpret_cast<Function>(address);
}
}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: probe_vtd_vehicle_definition /absolute/path/to/"
                     "libModuleTrafficDyn.so.2025.2.438\n"
                     "Offline only; requires matching VTD cxx11 dependency libraries "
                     "in LD_LIBRARY_PATH.\n";
        return 2;
    }
    try {
        if (sizeof(void*) != 8 || sha256(argv[1]) != kKnownSha256)
            throw std::runtime_error(
                "Unsupported plugin build; refusing version-specific internal reads");
        void* library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
        if (!library) throw std::runtime_error(dlerror());
        auto make = symbol<void* (*)()>(library, "makeModule");
        auto setPlayer = symbol<void (*)(void*, const std::string&)>(library,
            "_ZN6Module12ModulePlugin9setPlayerERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE");
        auto useDefault = symbol<void (*)(void*, bool)>(library,
            "_ZN6Module12ModulePlugin16useDefaultPlayerEb");
        auto handle = symbol<bool (*)(void*, const std::string&)>(library,
            "_ZN6Module10TrafficDyn16handleSCPCommandERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE");
        auto getDefinition = symbol<const void* (*)(void*)>(library,
            "_ZN6DynVeh7IDynVeh20getVehicleDefinitionEv");
        auto destroy = symbol<void (*)(void*)>(library,
            "_ZN6Module10TrafficDynD0Ev");
        bool reproduced = true;
        for (bool defaultPlayer : {true, false}) {
            std::unique_ptr<void, decltype(destroy)> object(make(), destroy);
            if (!object) throw std::runtime_error("Plugin factory returned null");
            setPlayer(object.get(), std::string("Ego"));
            useDefault(object.get(), defaultPlayer);
            std::cout << "defaultPlayer=" << defaultPlayer << " player=Ego\n";
            for (auto [name, wheelbase] : {
                     std::pair{"Ego", "2.944"},
                     std::pair{"NewPlayer09", "7.317"},
                     std::pair{"NewPlayer13", "3.696"}}) {
                const std::string command =
                    "<Traffic><Player name=\"" + std::string(name) +
                    "\"><VehicleDef wheelBase=\"" + wheelbase +
                    "\"/></Player></Traffic>";
                const bool parsed = handle(object.get(), command);
                void* model = nullptr;
                std::memcpy(&model,
                            static_cast<char*>(object.get()) + kModelOffset,
                            sizeof(model));
                if (!model) throw std::runtime_error("Null dynamics model");
                const void* definition = getDefinition(model);
                if (!definition) throw std::runtime_error("Null vehicle definition");
                double actualWheelbase = 0;
                std::memcpy(&actualWheelbase,
                            static_cast<const char*>(definition) + kWheelbaseOffset,
                            sizeof(actualWheelbase));
                const double expected = defaultPlayer ? std::stod(wheelbase) : 2.944;
                reproduced &= parsed && std::isfinite(actualWheelbase) &&
                              std::abs(actualWheelbase - expected) < 1e-5;
                std::cout << "incoming=" << name << " wheelbase=" << wheelbase
                          << " parsed=" << parsed
                          << " model_wheelbase=" << actualWheelbase << '\n';
            }
        }
        dlclose(library);
        std::cout << (reproduced ? "REPRODUCED" : "UNEXPECTED RESULT") << '\n';
        return reproduced ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
