#include <DataStreamClient.h>

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace {

using namespace ViconDataStreamSDK::CPP;

bool Ok(Result::Enum result) { return result == Result::Success; }

bool WaitFrame(Client* client, double timeout_s) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    if (Ok(client->GetFrame().Result)) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

void PrintVec3(const double v[3]) {
  std::cout << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
}

}  // namespace

int main(int argc, char** argv) {
  std::string host = "localhost:801";
  unsigned int max_items = 32;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    if (key == "--host" && i + 1 < argc) {
      host = argv[++i];
    } else if (key == "--max-items" && i + 1 < argc) {
      max_items = static_cast<unsigned int>(std::max(0, std::atoi(argv[++i])));
    } else if (key == "--help" || key == "-h") {
      std::cerr << "Usage: " << argv[0]
                << " [--host HOST[:PORT]] [--max-items N]\n";
      return 0;
    } else {
      std::cerr << "Unknown argument: " << key << "\n";
      return 2;
    }
  }

  Client client;
  client.SetStreamMode(StreamMode::ClientPull);

  std::cout << std::setprecision(17);
  std::cout << "Connecting to " << host << " ...\n";
  const auto connect = client.Connect(host);
  if (!Ok(connect.Result)) {
    std::cerr << "Connect failed result=" << static_cast<int>(connect.Result)
              << "\n";
    return 1;
  }

  client.EnableSegmentData();
  client.EnableMarkerData();
  client.EnableUnlabeledMarkerData();
  client.EnableDeviceData();
  client.EnableCentroidData();

  if (!WaitFrame(&client, 5.0)) {
    std::cerr << "Timed out waiting for frame\n";
    return 1;
  }

  const auto frame = client.GetFrameNumber();
  const auto rate = client.GetFrameRate();
  std::cout << "frame_number="
            << (Ok(frame.Result) ? std::to_string(frame.FrameNumber) : "N/A")
            << "\n";
  std::cout << "frame_rate_hz="
            << (Ok(rate.Result) ? std::to_string(rate.FrameRateHz) : "N/A")
            << "\n";

  const auto subjects = client.GetSubjectCount();
  std::cout << "\n[subjects] result=" << static_cast<int>(subjects.Result)
            << " count=" << subjects.SubjectCount << "\n";
  for (unsigned int si = 0; Ok(subjects.Result) && si < subjects.SubjectCount; ++si) {
    const auto subject = client.GetSubjectName(si);
    if (!Ok(subject.Result)) continue;
    const std::string subject_name = subject.SubjectName;
    const auto root = client.GetSubjectRootSegmentName(subject_name);
    std::cout << "subject[" << si << "] name=" << subject_name
              << " root=" << (Ok(root.Result) ? std::string(root.SegmentName) : "<err>")
              << "\n";

    const auto segments = client.GetSegmentCount(subject_name);
    std::cout << "  segments result=" << static_cast<int>(segments.Result)
              << " count=" << segments.SegmentCount << "\n";

    const auto markers = client.GetMarkerCount(subject_name);
    std::cout << "  subject_markers result=" << static_cast<int>(markers.Result)
              << " count=" << markers.MarkerCount << "\n";
    const unsigned int marker_limit = std::min(markers.MarkerCount, max_items);
    for (unsigned int mi = 0; Ok(markers.Result) && mi < marker_limit; ++mi) {
      const auto marker_name = client.GetMarkerName(subject_name, mi);
      if (!Ok(marker_name.Result)) continue;
      const auto marker_pos =
          client.GetMarkerGlobalTranslation(subject_name, marker_name.MarkerName);
      std::cout << "    marker[" << mi << "] name="
                << std::string(marker_name.MarkerName)
                << " result=" << static_cast<int>(marker_pos.Result)
                << " occluded=" << (Ok(marker_pos.Result) ? marker_pos.Occluded : true)
                << " pos_mm=";
      if (Ok(marker_pos.Result)) {
        PrintVec3(marker_pos.Translation);
      } else {
        std::cout << "<err>";
      }
      std::cout << "\n";
    }
  }

  const auto labeled = client.GetLabeledMarkerCount();
  std::cout << "\n[labeled_markers_global] result="
            << static_cast<int>(labeled.Result)
            << " count=" << labeled.MarkerCount << "\n";
  const unsigned int labeled_limit = std::min(labeled.MarkerCount, max_items);
  for (unsigned int i = 0; Ok(labeled.Result) && i < labeled_limit; ++i) {
    const auto pos = client.GetLabeledMarkerGlobalTranslation(i);
    const auto data = client.GetLabeledMarkerData(i);
    std::cout << "  labeled[" << i << "] pos_result="
              << static_cast<int>(pos.Result)
              << " data_result=" << static_cast<int>(data.Result);
    if (Ok(pos.Result)) {
      std::cout << " marker_id=" << pos.MarkerID << " pos_mm=";
      PrintVec3(pos.Translation);
    }
    if (Ok(data.Result)) {
      std::cout << " trajectory_id=" << data.TrajectoryID
                << " subject_id=" << data.SubjectID
                << " marker_id2=" << data.MarkerID;
    }
    std::cout << "\n";
  }

  const auto unlabeled = client.GetUnlabeledMarkerCount();
  std::cout << "\n[unlabeled_markers_global] result="
            << static_cast<int>(unlabeled.Result)
            << " count=" << unlabeled.MarkerCount << "\n";
  const unsigned int unlabeled_limit = std::min(unlabeled.MarkerCount, max_items);
  for (unsigned int i = 0; Ok(unlabeled.Result) && i < unlabeled_limit; ++i) {
    const auto pos = client.GetUnlabeledMarkerGlobalTranslation(i);
    std::cout << "  unlabeled[" << i << "] result="
              << static_cast<int>(pos.Result);
    if (Ok(pos.Result)) {
      std::cout << " marker_id=" << pos.MarkerID << " pos_mm=";
      PrintVec3(pos.Translation);
    }
    std::cout << "\n";
  }

  const auto devices = client.GetDeviceCount();
  std::cout << "\n[devices] result=" << static_cast<int>(devices.Result)
            << " count=" << devices.DeviceCount << "\n";
  for (unsigned int i = 0; Ok(devices.Result) && i < std::min(devices.DeviceCount, max_items); ++i) {
    const auto device = client.GetDeviceName(i);
    std::cout << "  device[" << i << "] result="
              << static_cast<int>(device.Result);
    if (Ok(device.Result)) {
      std::cout << " name=" << std::string(device.DeviceName)
                << " type=" << static_cast<int>(device.DeviceType);
    }
    std::cout << "\n";
  }

  const auto cameras = client.GetCameraCount();
  std::cout << "\n[cameras_centroids_2d] result="
            << static_cast<int>(cameras.Result)
            << " count=" << cameras.CameraCount << "\n";
  for (unsigned int ci = 0; Ok(cameras.Result) && ci < std::min(cameras.CameraCount, max_items); ++ci) {
    const auto camera = client.GetCameraName(ci);
    if (!Ok(camera.Result)) {
      std::cout << "  camera[" << ci << "] result="
                << static_cast<int>(camera.Result) << "\n";
      continue;
    }
    const std::string camera_name = camera.CameraName;
    const auto centroid_count = client.GetCentroidCount(camera_name);
    std::cout << "  camera[" << ci << "] name=" << camera_name
              << " centroid_result=" << static_cast<int>(centroid_count.Result)
              << " centroid_count=" << centroid_count.CentroidCount << "\n";
    const unsigned int centroid_limit =
        std::min(centroid_count.CentroidCount, std::min(max_items, 8u));
    for (unsigned int j = 0; Ok(centroid_count.Result) && j < centroid_limit; ++j) {
      const auto centroid = client.GetCentroidPosition(camera_name, j);
      std::cout << "    centroid[" << j << "] result="
                << static_cast<int>(centroid.Result);
      if (Ok(centroid.Result)) {
        std::cout << " xy=[" << centroid.CentroidPosition[0] << ", "
                  << centroid.CentroidPosition[1] << "] radius="
                  << centroid.Radius;
      }
      std::cout << "\n";
    }
  }

  client.Disconnect();
  return 0;
}
