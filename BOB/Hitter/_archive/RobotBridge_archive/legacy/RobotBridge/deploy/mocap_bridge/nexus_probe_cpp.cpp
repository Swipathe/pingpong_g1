#include "DataStreamClient.h"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

using namespace ViconDataStreamSDK::CPP;

namespace {

volatile std::sig_atomic_t g_stop = 0;

void HandleSignal(int) { g_stop = 1; }

struct Args {
  std::string host = "localhost:801";
  bool list = false;
  std::string ball_subject;
  std::string ball_marker;
  std::string base_subject;
  std::string base_segment;
  double duration_s = 0.0;
  double print_hz = 10.0;
  std::string csv_path;
};

void Usage(const char* argv0) {
  std::cout
      << "Usage:\n"
      << "  " << argv0 << " --host <nexus_ip:801> --list\n"
      << "  " << argv0 << " --host <nexus_ip:801> --ball-subject Ball --ball-marker Ball [--base-subject G1Base] [--base-segment root]\n\n"
      << "Options:\n"
      << "  --host HOST              Vicon DataStream host, default localhost:801\n"
      << "  --list                   List subjects, segments, and markers, then exit\n"
      << "  --ball-subject NAME      Subject containing the ball marker\n"
      << "  --ball-marker NAME       Ball marker name\n"
      << "  --base-subject NAME      Optional robot base subject\n"
      << "  --base-segment NAME      Optional base segment; defaults to subject root segment\n"
      << "  --duration SEC           Run duration; 0 means until Ctrl-C\n"
      << "  --print-hz HZ            Console print rate, default 10\n"
      << "  --csv PATH               Optional CSV output\n";
}

bool ParseArgs(int argc, char** argv, Args* args) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << key << "\n";
        std::exit(2);
      }
      return argv[++i];
    };

    if (key == "--help" || key == "-h") {
      Usage(argv[0]);
      std::exit(0);
    } else if (key == "--host") {
      args->host = next();
    } else if (key == "--list") {
      args->list = true;
    } else if (key == "--ball-subject") {
      args->ball_subject = next();
    } else if (key == "--ball-marker") {
      args->ball_marker = next();
    } else if (key == "--base-subject") {
      args->base_subject = next();
    } else if (key == "--base-segment") {
      args->base_segment = next();
    } else if (key == "--duration") {
      args->duration_s = std::stod(next());
    } else if (key == "--print-hz") {
      args->print_hz = std::stod(next());
    } else if (key == "--csv") {
      args->csv_path = next();
    } else {
      std::cerr << "Unknown argument: " << key << "\n";
      return false;
    }
  }
  return true;
}

bool Ok(Result::Enum result) { return result == Result::Success; }

bool WaitFrame(Client* client, double timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline && !g_stop) {
    const auto output = client->GetFrame();
    if (Ok(output.Result)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

void ListStream(Client* client) {
  if (!WaitFrame(client, 5.0)) {
    throw std::runtime_error("Timed out waiting for the first Vicon frame.");
  }

  const unsigned int subject_count = client->GetSubjectCount().SubjectCount;
  std::cout << "Subjects (" << subject_count << "):\n";
  for (unsigned int s = 0; s < subject_count; ++s) {
    const std::string subject = client->GetSubjectName(s).SubjectName;
    std::cout << "  " << subject << "\n";
    const auto root = client->GetSubjectRootSegmentName(subject);
    if (Ok(root.Result)) {
      std::cout << "    root segment: " << root.SegmentName << "\n";
    }

    const unsigned int segment_count = client->GetSegmentCount(subject).SegmentCount;
    std::cout << "    segments (" << segment_count << "): ";
    for (unsigned int i = 0; i < segment_count; ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << client->GetSegmentName(subject, i).SegmentName;
    }
    std::cout << "\n";

    const unsigned int marker_count = client->GetMarkerCount(subject).MarkerCount;
    std::cout << "    markers (" << marker_count << "): ";
    for (unsigned int i = 0; i < marker_count; ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << client->GetMarkerName(subject, i).MarkerName;
    }
    std::cout << "\n";
  }
}

void WriteCsvHeader(std::ofstream* csv) {
  *csv << "host_time_s,frame_number,ball_valid,ball_occluded,"
       << "ball_x_mm,ball_y_mm,ball_z_mm,ball_x_m,ball_y_m,ball_z_m,"
       << "base_valid,base_occluded,base_x_mm,base_y_mm,base_z_mm,"
       << "base_qx,base_qy,base_qz,base_qw\n";
}

int Run(const Args& args) {
  Client client;
  client.SetStreamMode(StreamMode::ClientPull);

  std::cout << "Connecting to " << args.host << " ...\n";
  const auto connect = client.Connect(args.host);
  if (!Ok(connect.Result)) {
    std::cerr << "Failed to connect: " << static_cast<int>(connect.Result) << "\n";
    return 1;
  }

  client.EnableMarkerData();
  client.EnableSegmentData();
  client.EnableUnlabeledMarkerData();

  if (args.list) {
    ListStream(&client);
    client.Disconnect();
    return 0;
  }

  if (args.ball_subject.empty() || args.ball_marker.empty()) {
    std::cerr << "--ball-subject and --ball-marker are required unless --list is used.\n";
    client.Disconnect();
    return 2;
  }

  std::string base_segment = args.base_segment;
  if (!args.base_subject.empty() && base_segment.empty()) {
    if (!WaitFrame(&client, 5.0)) {
      std::cerr << "Timed out waiting for frame while resolving base root segment.\n";
      client.Disconnect();
      return 1;
    }
    const auto root = client.GetSubjectRootSegmentName(args.base_subject);
    if (!Ok(root.Result)) {
      std::cerr << "Could not resolve root segment for base subject " << args.base_subject << "\n";
      client.Disconnect();
      return 1;
    }
    base_segment = root.SegmentName;
    std::cout << "Using root segment for base subject `" << args.base_subject << "`: " << base_segment << "\n";
  }

  std::ofstream csv;
  if (!args.csv_path.empty()) {
    csv.open(args.csv_path);
    if (!csv) {
      std::cerr << "Could not open CSV for writing: " << args.csv_path << "\n";
      client.Disconnect();
      return 1;
    }
    WriteCsvHeader(&csv);
    std::cout << "Writing samples to " << args.csv_path << "\n";
  }

  const auto start = std::chrono::steady_clock::now();
  auto last_print = start - std::chrono::seconds(10);
  unsigned long frames = 0;

  while (!g_stop) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start).count();
    if (args.duration_s > 0.0 && elapsed >= args.duration_s) {
      break;
    }

    if (!WaitFrame(&client, 2.0)) {
      std::cerr << "Timed out waiting for Vicon frame.\n";
      continue;
    }
    ++frames;

    const auto frame_number = client.GetFrameNumber();
    const auto ball = client.GetMarkerGlobalTranslation(args.ball_subject, args.ball_marker);
    const bool ball_valid = Ok(ball.Result) && !ball.Occluded;

    bool base_enabled = !args.base_subject.empty() && !base_segment.empty();
    Output_GetSegmentGlobalTranslation base_translation;
    Output_GetSegmentGlobalRotationQuaternion base_rotation;
    bool base_valid = false;
    if (base_enabled) {
      base_translation = client.GetSegmentGlobalTranslation(args.base_subject, base_segment);
      base_rotation = client.GetSegmentGlobalRotationQuaternion(args.base_subject, base_segment);
      base_valid = Ok(base_translation.Result) && !base_translation.Occluded && Ok(base_rotation.Result) && !base_rotation.Occluded;
    }

    const double host_time_s = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    if (csv) {
      csv << std::fixed << std::setprecision(9)
          << host_time_s << ","
          << frame_number.FrameNumber << ","
          << static_cast<int>(ball_valid) << ","
          << static_cast<int>(ball.Occluded) << ","
          << ball.Translation[0] << "," << ball.Translation[1] << "," << ball.Translation[2] << ","
          << ball.Translation[0] * 0.001 << "," << ball.Translation[1] * 0.001 << "," << ball.Translation[2] * 0.001 << ",";
      if (base_enabled) {
        csv << static_cast<int>(base_valid) << ","
            << static_cast<int>(base_translation.Occluded) << ","
            << base_translation.Translation[0] << "," << base_translation.Translation[1] << "," << base_translation.Translation[2] << ","
            << base_rotation.Rotation[0] << "," << base_rotation.Rotation[1] << ","
            << base_rotation.Rotation[2] << "," << base_rotation.Rotation[3] << "\n";
      } else {
        csv << ",,,,,,,,\n";
      }
    }

    const double print_period = 1.0 / std::max(args.print_hz, 1.0e-6);
    if (std::chrono::duration<double>(now - last_print).count() >= print_period) {
      const double hz = frames / std::max(elapsed, 1.0e-6);
      std::cout << std::fixed << std::setprecision(4)
                << "frame=" << frame_number.FrameNumber
                << " stream_hz=" << std::setprecision(1) << hz
                << " ball_valid=" << ball_valid
                << " ball_m=[" << std::setprecision(4)
                << ball.Translation[0] * 0.001 << ", "
                << ball.Translation[1] * 0.001 << ", "
                << ball.Translation[2] * 0.001 << "]";
      if (base_enabled) {
        std::cout << " base_valid=" << base_valid
                  << " base_m=["
                  << base_translation.Translation[0] * 0.001 << ", "
                  << base_translation.Translation[1] * 0.001 << ", "
                  << base_translation.Translation[2] * 0.001 << "]"
                  << " base_quat=["
                  << base_rotation.Rotation[0] << ", "
                  << base_rotation.Rotation[1] << ", "
                  << base_rotation.Rotation[2] << ", "
                  << base_rotation.Rotation[3] << "]";
      }
      std::cout << "\n";
      last_print = now;
    }
  }

  client.Disconnect();
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  Args args;
  if (!ParseArgs(argc, argv, &args)) {
    Usage(argv[0]);
    return 2;
  }

  try {
    return Run(args);
  } catch (const std::exception& exc) {
    std::cerr << "ERROR: " << exc.what() << "\n";
    return 1;
  }
}
