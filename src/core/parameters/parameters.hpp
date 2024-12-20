#pragma once

#include <map>
#include <string>

namespace rmcs_auto_aim {
class Parameters {
protected:
    Parameters() = default;

public:
    Parameters(const Parameters&)            = delete;
    Parameters& operator=(const Parameters&) = delete;
    Parameters(Parameters&&)                 = delete;

    static Parameters& getInstance() {
        static Parameters instance_;
        return instance_;
    }

    void setBoolParam(const std::string& name, bool value) { bool_params_[name] = value; }

    bool getBoolParam(const std::string& name) { return bool_params_[name]; }

private:
    std::map<std::string, bool> bool_params_;
};
} // namespace rmcs_auto_aim