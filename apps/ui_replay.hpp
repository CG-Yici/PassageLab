#pragma once
#include <map>

// Input-driven integration checks use the same widgets and application handlers
// as an interactive session. No operating-system mouse/keyboard injection.
struct UiReplay {
    struct Step { std::string label; bool drag = false, shift = false; float wheel = 0; };
    std::vector<Step> steps;
    std::map<std::string, ImVec2> controls;
    std::function<bool(std::size_t)> verify;
    pl::Json completed = pl::Json::array();
    std::size_t stage = 0;
    int phase = 0, settle = 0;
    ImVec2 point{};
    std::chrono::steady_clock::time_point since = std::chrono::steady_clock::now();
    void track(const char *label) {
        if (!ImGui::IsItemVisible()) return;
        auto a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
        // Use the first component when the target is a vector editor.
        controls.try_emplace(label, ImVec2(a.x + std::min(40.f, (b.x-a.x)*.5f), (a.y+b.y)*.5f));
    }
    void frame(ImGuiIO &io) {
        if (stage == steps.size()) return;
        if (std::chrono::steady_clock::now() - since > std::chrono::seconds(20))
            throw std::runtime_error("Input replay timed out: " + steps[stage].label);
        const auto &step = steps[stage];
        if (phase == 0) {
            auto it = controls.find(step.label);
            if (it == controls.end()) return;
            point = it->second;
            io.AddMousePosEvent(point.x, point.y);
            if (step.wheel != 0) {
                io.AddMouseWheelEvent(0, step.wheel);
                phase = 3;
                return;
            }
            if (step.shift) io.AddKeyEvent(ImGuiMod_Shift, true);
            io.AddMouseButtonEvent(0, true);
            phase = 1;
        } else if (phase == 1) {
            if (step.drag) io.AddMousePosEvent(point.x + 24, point.y);
            else io.AddMouseButtonEvent(0, false);
            phase = step.drag ? 2 : 3;
        } else if (phase == 2) {
            io.AddMouseButtonEvent(0, false);
            if (step.shift) io.AddKeyEvent(ImGuiMod_Shift, false);
            phase = 3;
        } else if (++settle >= 3 && verify(stage)) {
            completed.push_back({{"control", step.label}, {"passed", true}});
            ++stage;
            phase = settle = 0;
            since = std::chrono::steady_clock::now();
        }
    }
};
inline UiReplay *active_replay = nullptr;
inline bool ui_button(const char *label, const ImVec2 &size = ImVec2(0, 0)) {
    const bool clicked = ImGui::Button(label, size);
    if (active_replay) active_replay->track(label);
    return clicked;
}
inline bool ui_checkbox(const char *label, bool *value) {
    const bool changed = ImGui::Checkbox(label, value);
    if (active_replay) active_replay->track(label);
    return changed;
}
