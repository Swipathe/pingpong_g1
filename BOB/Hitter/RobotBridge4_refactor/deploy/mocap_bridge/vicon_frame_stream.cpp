#include <DataStreamClient.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace ViconDataStreamSDK::CPP;

std::atomic<bool> g_stop{false};

void SignalHandler(int) { g_stop.store(true); }

bool Ok(Result::Enum result) { return result == Result::Success; }

struct Args {
    std::string host = "localhost:801";
    std::string base_subject = "G2Pelvis";
    double duration_s = 0.0;
    double source_rate_hz = 300.0;
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

void PrintUsage(const char *argv0) {
    std::cerr << "Usage: " << argv0 << " [--host HOST[:PORT]] [--tracker-name NAME]"
              << " [--base-subject SUBJECT]"
              << " [--source-rate-hz HZ] [--duration SEC]\n";
}

bool ParseArgs(int argc, char **argv, Args *args) {
    for (int i = 1; i < argc; ++i) {
        const std::string key(argv[i]);
        auto need_value = [&](const char *name) -> const char * {
            if (i + 1 >= argc) {
                std::cerr << name << " requires a value\n";
                return nullptr;
            }
            return argv[++i];
        };
        if (key == "--host") {
            const char *value = need_value("--host");
            if (value == nullptr)
                return false;
            args->host = value;
        } else if (key == "--base-subject" || key == "--tracker-name") {
            const char *value = need_value(key.c_str());
            if (value == nullptr)
                return false;
            args->base_subject = value;
        } else if (key == "--source-rate-hz") {
            const char *value = need_value("--source-rate-hz");
            if (value == nullptr)
                return false;
            args->source_rate_hz = std::atof(value);
        } else if (key == "--duration") {
            const char *value = need_value("--duration");
            if (value == nullptr)
                return false;
            args->duration_s = std::atof(value);
        } else if (key == "--help" || key == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        } else {
            std::cerr << "Unknown argument: " << key << "\n";
            return false;
        }
    }
    if (!std::isfinite(args->source_rate_hz) || args->source_rate_hz <= 0.0) {
        std::cerr << "--source-rate-hz must be finite and positive\n";
        return false;
    }
    if (!std::isfinite(args->duration_s) || args->duration_s < 0.0) {
        std::cerr << "--duration must be finite and nonnegative\n";
        return false;
    }
    return true;
}

bool WaitFrame(Client *client, double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline && !g_stop.load()) {
        if (Ok(client->GetFrame().Result))
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

void WriteNullableVec3(std::ostream &out, bool valid, const Vec3 &value) {
    if (!valid) {
        out << "null";
        return;
    }
    out << "[" << value.x << "," << value.y << "," << value.z << "]";
}

void WriteNullableQuat(std::ostream &out, bool valid, const double q[4]) {
    if (!valid) {
        out << "null";
        return;
    }
    out << "[" << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "]";
}

void WriteUnlabeledMarkers(std::ostream &out, Client *client) {
    const auto count_output = client->GetUnlabeledMarkerCount();
    if (!Ok(count_output.Result)) {
        out << "[]";
        return;
    }
    out << "[";
    bool first = true;
    for (unsigned int i = 0; i < count_output.MarkerCount; ++i) {
        const auto marker = client->GetUnlabeledMarkerGlobalTranslation(i);
        if (!Ok(marker.Result))
            continue;
        const Vec3 p{
            marker.Translation[0],
            marker.Translation[1],
            marker.Translation[2],
        };
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        if (!first)
            out << ",";
        out << "[" << p.x << "," << p.y << "," << p.z << "]";
        first = false;
    }
    out << "]";
}

} // namespace

int main(int argc, char **argv) {
    Args args;
    if (!ParseArgs(argc, argv, &args)) {
        PrintUsage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    Client client;
    client.SetStreamMode(StreamMode::ClientPull);

    std::cerr << "Connecting to Vicon " << args.host << " ...\n";
    const auto connect = client.Connect(args.host);
    if (!Ok(connect.Result)) {
        std::cerr << "Connect failed: " << static_cast<int>(connect.Result) << "\n";
        return 1;
    }

    client.EnableSegmentData();
    client.EnableUnlabeledMarkerData();

    if (!WaitFrame(&client, 5.0)) {
        std::cerr << "Timed out waiting for initial Vicon frame.\n";
        return 1;
    }

    const auto root = client.GetSubjectRootSegmentName(args.base_subject);
    if (!Ok(root.Result) || std::string(root.SegmentName).empty()) {
        std::cerr << "Could not resolve root segment for " << args.base_subject << "\n";
        return 1;
    }
    const std::string base_segment = root.SegmentName;
    std::cerr << "Using root segment for " << args.base_subject << ": " << base_segment << "\n";

    const auto sdk_rate = client.GetFrameRate();
    double source_rate_hz = args.source_rate_hz;
    if (Ok(sdk_rate.Result) && std::isfinite(sdk_rate.FrameRateHz) && sdk_rate.FrameRateHz > 0.0) {
        source_rate_hz = sdk_rate.FrameRateHz;
    }
    std::cerr << "Vicon source rate=" << source_rate_hz << " Hz\n";

    const auto start = std::chrono::steady_clock::now();
    std::int64_t fallback_frame_number = 0;

    std::cout << std::setprecision(17);
    while (!g_stop.load()) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        if (args.duration_s > 0.0 && elapsed >= args.duration_s)
            break;

        if (!Ok(client.GetFrame().Result)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const auto frame_number_output = client.GetFrameNumber();
        std::int64_t frame_number = ++fallback_frame_number;
        if (Ok(frame_number_output.Result)) {
            frame_number = static_cast<std::int64_t>(frame_number_output.FrameNumber);
            fallback_frame_number = frame_number;
        }
        const double source_time_s =
            source_rate_hz > 0.0 ? static_cast<double>(frame_number) / source_rate_hz : elapsed;

        const auto translation = client.GetSegmentGlobalTranslation(args.base_subject, base_segment);
        const auto rotation = client.GetSegmentGlobalRotationQuaternion(args.base_subject, base_segment);
        const bool body_valid =
            Ok(translation.Result) && !translation.Occluded && Ok(rotation.Result) && !rotation.Occluded;
        const Vec3 body_position_mm{
            translation.Translation[0],
            translation.Translation[1],
            translation.Translation[2],
        };
        const double body_quaternion_xyzw[4] = {
            rotation.Rotation[0],
            rotation.Rotation[1],
            rotation.Rotation[2],
            rotation.Rotation[3],
        };

        std::cout << "{\"frame_number\":" << frame_number << ",\"source_time_s\":" << source_time_s
                  << ",\"body_position_mm\":";
        WriteNullableVec3(std::cout, body_valid, body_position_mm);
        std::cout << ",\"body_quaternion_xyzw\":";
        WriteNullableQuat(std::cout, body_valid, body_quaternion_xyzw);
        std::cout << ",\"body_markers_mm\":{},\"unlabeled_markers_mm\":";
        WriteUnlabeledMarkers(std::cout, &client);
        std::cout << "}\n";
        std::cout.flush();
    }

    client.Disconnect();
    return 0;
}
