/*
 * VDL Mode 2 decoder module for SDR++.
 *
 * Channelises a VDL2 voice-channel-width slice of spectrum to 105 kHz complex
 * baseband and feeds it to a self-contained D8PSK / AVLC / ACARS decoder
 * (see src/vdl2/). Decoded frames are shown in a detachable message window
 * and can be logged to a TSV file.
 */
#include <imgui.h>
#include <config.h>
#include <core.h>
#include <gui/style.h>
#include <gui/gui.h>
#include <gui/tuner.h>
#include <signal_path/signal_path.h>
#include <module.h>
#include <gui/widgets/folder_select.h>
#include <utils/optionlist.h>
#include <dsp/sink/handler_sink.h>

#include <ctime>
#include <cstdio>
#include <mutex>
#include <vector>
#include <string>

#include "vdl2/vdl2.h"
#include "vdl2/avlc.h"

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "vdl2_decoder",
    /* Description:     */ "VDL Mode 2 (aviation VHF Data Link) decoder",
    /* Author:          */ "SDR++ Community",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ -1
};

ConfigManager config;

// VFO output rate required by the VDL2 demodulator (SPS * symbol rate).
#define VDL2_SAMPLE_RATE   ((double)vdl2::SAMPLE_RATE) // 105000 Hz
#define VDL2_BANDWIDTH     14000.0

// One decoded message as shown in the table / written to the log.
struct MsgEntry {
    std::string time;
    std::string src;
    std::string dst;
    std::string typ;    // S->D address types
    std::string ag;     // Air / Ground
    std::string proto;
    std::string label;
    std::string flight;
    std::string text;
    int corr = 0;
};

class VDL2DecoderModule : public ModuleManager::Instance {
public:
    VDL2DecoderModule(std::string name) : folderSelect("%ROOT%/recordings") {
        this->name = name;

        // Standard VDL2 channels (kHz in the 136 MHz aeronautical band).
        channels.define("136.975 (CSC)", 136975000.0);
        channels.define("136.725",       136725000.0);
        channels.define("136.775",       136775000.0);
        channels.define("136.825",       136825000.0);
        channels.define("136.875",       136875000.0);
        channels.define("136.700",       136700000.0);
        channels.define("136.800",       136800000.0);
        channels.define("136.925",       136925000.0);

        // Restore config.
        config.acquire();
        if (config.conf[name].contains("showWindow"))  { showWindow  = config.conf[name]["showWindow"];  }
        if (config.conf[name].contains("autoScroll"))  { autoScroll  = config.conf[name]["autoScroll"];  }
        if (config.conf[name].contains("recording"))   { recording   = config.conf[name]["recording"];   }
        if (config.conf[name].contains("recordPath"))  { folderSelect.setPath(config.conf[name]["recordPath"]); }
        if (config.conf[name].contains("channelId")) {
            std::string cn = config.conf[name]["channelId"];
            if (channels.keyExists(cn)) { chanId = channels.keyId(cn); }
        }
        config.release();

        // Build the decoder. The frame callback runs on the DSP thread.
        demod = std::make_unique<vdl2::Demodulator>(
            [this](const vdl2::RawFrame& rf){ this->onRawFrame(rf); });

        // VFO + sink.
        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER,
                                            0, VDL2_BANDWIDTH, VDL2_SAMPLE_RATE,
                                            VDL2_BANDWIDTH, VDL2_BANDWIDTH, true);
        vfo->setSnapInterval(25000);
        sink.init(vfo->output, _sinkHandler, this);
        sink.start();

        gui::menu.registerEntry(name, menuHandler, this, this);
    }

    ~VDL2DecoderModule() {
        gui::menu.removeEntry(name);
        if (enabled) {
            sink.stop();
            sigpath::vfoManager.deleteVFO(vfo);
        }
        closeLog();
        sigpath::sinkManager.unregisterStream(name);
    }

    void postInit() {}

    void enable() {
        double bw = gui::waterfall.getBandwidth();
        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER,
                                            std::clamp<double>(0, -bw / 2.0, bw / 2.0),
                                            VDL2_BANDWIDTH, VDL2_SAMPLE_RATE,
                                            VDL2_BANDWIDTH, VDL2_BANDWIDTH, true);
        vfo->setSnapInterval(25000);
        sink.setInput(vfo->output);
        sink.start();
        enabled = true;
    }

    void disable() {
        sink.stop();
        sigpath::vfoManager.deleteVFO(vfo);
        enabled = false;
    }

    bool isEnabled() { return enabled; }

private:
    // ---- DSP path ----
    static void _sinkHandler(dsp::complex_t* data, int count, void* ctx) {
        VDL2DecoderModule* _this = (VDL2DecoderModule*)ctx;
        // dsp::complex_t is two consecutive floats (re, im); pass straight through.
        _this->demod->process(reinterpret_cast<const float*>(data), count);
    }

    // Called from the DSP thread when a frame is de-framed.
    void onRawFrame(const vdl2::RawFrame& rf) {
        vdl2::Frame f = vdl2::parse_avlc(rf);
        if (!f.valid || !f.fcs_ok) { return; }
        // Drop ACARS frames whose sub-block fails structural/BCS validation,
        // as dumpvdl2 does (these would otherwise show as garbage rows).
        if (f.acars_malformed) { return; }

        MsgEntry e;
        char tbuf[16];
        time_t t = time(nullptr);
        struct tm* lt = localtime(&t);
        strftime(tbuf, sizeof(tbuf), "%H:%M:%S", lt);
        e.time   = tbuf;
        char ab[16];
        snprintf(ab, sizeof(ab), "%06X", f.src_addr & 0xFFFFFF); e.src = ab;
        snprintf(ab, sizeof(ab), "%06X", f.dst_addr & 0xFFFFFF); e.dst = ab;
        e.typ    = f.srcTypeStr() + "->" + f.dstTypeStr();
        e.ag     = f.airborne ? "Air" : "Gnd";
        e.proto  = f.proto;
        e.label  = f.acars_label;
        e.flight = f.acars_flight;
        e.corr   = f.fec_corrections;
        if (f.has_acars) { e.text = f.acars_text; }

        {
            std::lock_guard<std::mutex> lck(msgMtx);
            messages.push_back(e);
            if (messages.size() > 4096) { messages.erase(messages.begin()); }
            newData = true;
        }
        writeLog(e);
    }

    // ---- TSV logging ----
    void openLog() {
        if (logFile || !folderSelect.pathIsValid()) { return; }
        std::string p = folderSelect.path + "/vdl2_log.tsv";
        logFile = fopen(p.c_str(), "a");
        if (logFile) {
            fprintf(logFile, "time\tsrc\tdst\ttype\tag\tproto\tlabel\tflight\tcorr\ttext\n");
            fflush(logFile);
        }
    }
    void closeLog() {
        if (logFile) { fclose(logFile); logFile = nullptr; }
    }
    // Replace any character that would break TSV column structure.
    static std::string tsvSafe(const std::string& in) {
        std::string o; o.reserve(in.size());
        for (unsigned char c : in) {
            if (c == '\t' || c == '\n' || c == '\r') { o += ' '; }
            else if (c < 0x20 || c == 0x7f) { /* drop other control chars */ }
            else { o += (char)c; }
        }
        return o;
    }
    void writeLog(const MsgEntry& e) {
        std::lock_guard<std::mutex> lck(logMtx);
        if (!recording || !logFile) { return; }
        fprintf(logFile, "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%d\t%s\n",
                tsvSafe(e.time).c_str(), tsvSafe(e.src).c_str(), tsvSafe(e.dst).c_str(),
                tsvSafe(e.typ).c_str(), tsvSafe(e.ag).c_str(), tsvSafe(e.proto).c_str(),
                tsvSafe(e.label).c_str(), tsvSafe(e.flight).c_str(),
                e.corr, tsvSafe(e.text).c_str());
        fflush(logFile);
    }

    // ---- GUI ----
    static void menuHandler(void* ctx) {
        VDL2DecoderModule* _this = (VDL2DecoderModule*)ctx;
        float menuWidth = ImGui::GetContentRegionAvail().x;

        if (!_this->enabled) { style::beginDisabled(); }

        ImGui::LeftLabel("Channel");
        ImGui::FillWidth();
        if (ImGui::Combo(("##vdl2_chan_" + _this->name).c_str(), &_this->chanId, _this->channels.txt)) {
            tuner::tune(tuner::TUNER_MODE_NORMAL, _this->name, _this->channels.value(_this->chanId));
            config.acquire();
            config.conf[_this->name]["channelId"] = _this->channels.key(_this->chanId);
            config.release(true);
        }

        ImGui::LeftLabel("Noise floor");
        ImGui::Text("%.1f dBFS", _this->demod ? _this->demod->noiseFloorDbfs() : 0.0f);

        if (ImGui::Checkbox(("Log to file##vdl2_rec_" + _this->name).c_str(), &_this->recording)) {
            if (_this->recording) { _this->openLog(); } else { _this->closeLog(); }
            config.acquire();
            config.conf[_this->name]["recording"] = _this->recording;
            config.release(true);
        }
        if (_this->folderSelect.render("##vdl2_dir_" + _this->name)) {
            if (_this->folderSelect.pathIsValid()) {
                config.acquire();
                config.conf[_this->name]["recordPath"] = _this->folderSelect.path;
                config.release(true);
                if (_this->recording) { _this->closeLog(); _this->openLog(); }
            }
        }

        if (ImGui::Button(("Show Messages##vdl2_show_" + _this->name).c_str(), ImVec2(menuWidth, 0))) {
            _this->showWindow = true;
            config.acquire();
            config.conf[_this->name]["showWindow"] = true;
            config.release(true);
        }

        if (!_this->enabled) { style::endDisabled(); }

        if (_this->showWindow) { _this->drawWindow(); }
    }

    void drawWindow() {
        ImGui::SetNextWindowSize(ImVec2(900, 400), ImGuiCond_FirstUseEver);
        std::string title = "VDL2 Messages##" + name;
        bool open = ImGui::Begin(title.c_str(), &showWindow);

        // Keep the waterfall VFO from being retuned while the user interacts
        // with this window. IsWindowHovered() alone returns false whenever an
        // item is active (column-resize, button press) or when the cursor
        // leaves the window rect during a resize drag, so we also allow the
        // "blocked by active item" case and treat a focused window as locked.
        // This must run even when the window is collapsed (Begin == false),
        // since a collapsed window can still be moved/resized.
        ImGuiHoveredFlags hf = ImGuiHoveredFlags_RootAndChildWindows |
                               ImGuiHoveredFlags_AllowWhenBlockedByActiveItem;
        if (ImGui::IsWindowHovered(hf) ||
            ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
            gui::mainWindow.lockWaterfallControls = true;
        }

        if (!open) { ImGui::End(); return; }

        if (ImGui::Button(("Clear##vdl2_clr_" + name).c_str())) {
            std::lock_guard<std::mutex> lck(msgMtx);
            messages.clear();
        }
        ImGui::SameLine();
        ImGui::Checkbox(("Auto-scroll##vdl2_as_" + name).c_str(), &autoScroll);
        ImGui::SameLine();
        {
            std::lock_guard<std::mutex> lck(msgMtx);
            ImGui::Text("  %zu frames", messages.size());
        }

        const int COLS = 9;
        if (ImGui::BeginTable(("vdl2_tbl_" + name).c_str(), COLS,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable)) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time",   ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Src",    ImGuiTableColumnFlags_WidthFixed, 60);
            ImGui::TableSetupColumn("Dst",    ImGuiTableColumnFlags_WidthFixed, 60);
            ImGui::TableSetupColumn("Type",   ImGuiTableColumnFlags_WidthFixed, 150);
            ImGui::TableSetupColumn("A/G",    ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("Proto",  ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Label",  ImGuiTableColumnFlags_WidthFixed, 50);
            ImGui::TableSetupColumn("Flight", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Text",   ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            std::lock_guard<std::mutex> lck(msgMtx);
            for (auto& e : messages) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn(); ImGui::TextUnformatted(e.time.c_str());
                ImGui::TableNextColumn(); ImGui::TextUnformatted(e.src.c_str());
                ImGui::TableNextColumn(); ImGui::TextUnformatted(e.dst.c_str());
                ImGui::TableNextColumn(); ImGui::TextUnformatted(e.typ.c_str());
                ImGui::TableNextColumn(); ImGui::TextUnformatted(e.ag.c_str());
                ImGui::TableNextColumn(); ImGui::TextUnformatted(e.proto.c_str());
                ImGui::TableNextColumn(); ImGui::TextUnformatted(e.label.c_str());
                ImGui::TableNextColumn(); ImGui::TextUnformatted(e.flight.c_str());
                ImGui::TableNextColumn(); ImGui::TextUnformatted(e.text.c_str());
            }
            if (autoScroll && newData) {
                ImGui::SetScrollHereY(1.0f);
                newData = false;
            }
            ImGui::EndTable();
        }

        ImGui::End();

        // Persist window-closed state.
        static bool lastShow = true;
        if (lastShow && !showWindow) {
            config.acquire();
            config.conf[name]["showWindow"] = false;
            config.release(true);
        }
        lastShow = showWindow;
    }

    std::string name;
    bool enabled = true;

    VFOManager::VFO* vfo = nullptr;
    dsp::sink::Handler<dsp::complex_t> sink;
    std::unique_ptr<vdl2::Demodulator> demod;

    OptionList<std::string, double> channels;
    int chanId = 0;

    std::mutex msgMtx;
    std::vector<MsgEntry> messages;
    bool newData = false;

    bool showWindow = false;
    bool autoScroll = true;

    FolderSelect folderSelect;
    bool recording = false;
    std::mutex logMtx;
    FILE* logFile = nullptr;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    config.setPath(core::args["root"].s() + "/vdl2_decoder_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new VDL2DecoderModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (VDL2DecoderModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
