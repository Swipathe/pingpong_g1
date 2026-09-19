#define VICON_TABLE_LCM_BRIDGE_TESTING
#include "../vicon_table_lcm_bridge.cpp"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

void Expect(bool condition, const std::string &message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

BallCandidate Candidate(double x, double y, double z) { return BallCandidate{{}, {x, y, z}}; }

TableFrame IdentityTableForBallRoi() {
    TableFrame table;
    table.valid = true;
    table.center_raw_mm = {};
    table.x_axis_raw = {1.0, 0.0, 0.0};
    table.y_axis_raw = {0.0, 1.0, 0.0};
    table.z_axis_raw = {0.0, 0.0, 1.0};
    table.corners_raw_mm = {
        Vec3{10000.0, 10000.0, 10000.0},
        Vec3{11000.0, 10000.0, 10000.0},
        Vec3{10000.0, 11000.0, 10000.0},
        Vec3{11000.0, 11000.0, 10000.0},
    };
    return table;
}

Vec3 RawForTableWorld(const Vec3 &world, const Args &args) {
    return {
        (world.x - 0.5 * args.table_length_m) * 1000.0,
        world.y * 1000.0,
        (world.z - args.table_height_m) * 1000.0,
    };
}

bool Finite(const Vec3 &value) { return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z); }

enum class ExpectedIdRelation { Zero, Positive, Same, Greater };

struct TrackCase {
    const char *name;
    std::vector<BallCandidate> candidates;
    int64_t frame;
    double time;
    BallTrackPhase expected_phase;
    ExpectedIdRelation expected_id_relation;
    bool publish_valid;
    bool publish_end;
};

void CheckCases(BallTrackState *state, const std::vector<TrackCase> &cases, int64_t allocation_time_us = 1000000) {
    int64_t reference_id = state->track_id;
    for (size_t index = 0; index < cases.size(); ++index) {
        const TrackCase &test = cases[index];
        const BallTrackUpdate update = AdvanceBallTrack(state, test.candidates, test.frame, test.time, 0.35, 0.25,
                                                        allocation_time_us + static_cast<int64_t>(index));
        Expect(update.phase == test.expected_phase, std::string(test.name) + ": phase");
        Expect(update.publish_valid == test.publish_valid, std::string(test.name) + ": publish_valid");
        Expect(update.publish_end == test.publish_end, std::string(test.name) + ": publish_end");
        Expect(Finite(update.position), std::string(test.name) + ": finite position");
        switch (test.expected_id_relation) {
        case ExpectedIdRelation::Zero:
            Expect(update.track_id == 0, std::string(test.name) + ": zero id");
            break;
        case ExpectedIdRelation::Positive:
            Expect(update.track_id > 0, std::string(test.name) + ": positive id");
            reference_id = update.track_id;
            break;
        case ExpectedIdRelation::Same:
            Expect(update.track_id == reference_id, std::string(test.name) + ": same id");
            break;
        case ExpectedIdRelation::Greater:
            Expect(update.track_id > reference_id, std::string(test.name) + ": strictly greater id");
            reference_id = update.track_id;
            break;
        }
    }
}

void TestTrackIdentityAndGrace() {
    BallTrackState state;
    const auto first = AdvanceBallTrack(&state, {Candidate(0.80, -0.10, 1.00)}, 100, 1.00, 0.35, 0.25, 1000000);
    Expect(first.publish_valid && first.track_id == 1000000, "first eligible marker allocates injected positive id");
    const auto approach = AdvanceBallTrack(&state, {Candidate(0.50, -0.10, 1.00)}, 101, 1.01, 0.35, 0.25, 1000001);
    Expect(approach.publish_valid && approach.track_id == first.track_id,
           "approach establishes velocity within the strict gate");
    const auto reverse = AdvanceBallTrack(&state, {Candidate(-0.02, -0.10, 1.00)}, 102, 1.02, 0.35, 0.25, 1000002);
    Expect(reverse.publish_valid && reverse.track_id == first.track_id, "x<=0 and reversal retain id");
    const auto short_miss = AdvanceBallTrack(&state, {}, 120, 1.20, 0.35, 0.25, 1000003);
    Expect(!short_miss.publish_valid && !short_miss.publish_end, "short miss stays in grace");
    const auto end = AdvanceBallTrack(&state, {}, 127, 1.27, 0.35, 0.25, 1000004);
    Expect(end.publish_end && end.track_id == first.track_id, "timeout emits old id once");
    const auto end_again = AdvanceBallTrack(&state, {}, 128, 1.28, 0.35, 0.25, 1000005);
    Expect(!end_again.publish_end, "end is not repeated");
}

void TestTableDrivenTrackContract() {
    {
        BallTrackState state;
        CheckCases(&state, {
                               {"inactive rejects x<=0",
                                {Candidate(0.0, 0.0, 1.0)},
                                99,
                                0.99,
                                BallTrackPhase::Inactive,
                                ExpectedIdRelation::Zero,
                                false,
                                false},
                               {"start",
                                {Candidate(0.80, 0.0, 1.0)},
                                100,
                                1.00,
                                BallTrackPhase::Active,
                                ExpectedIdRelation::Positive,
                                true,
                                false},
                               {"distance exactly radius reassociates",
                                {Candidate(1.15, 0.0, 1.0)},
                                101,
                                1.01,
                                BallTrackPhase::Active,
                                ExpectedIdRelation::Same,
                                true,
                                false},
                           });
    }
    {
        BallTrackState state;
        AdvanceBallTrack(&state, {Candidate(0.80, 0.0, 1.0)}, 100, 1.00, 0.35, 0.25, 1100000);
        CheckCases(&state,
                   {
                       {"distance above radius does not take over",
                        {Candidate(1.150001, 0.0, 1.0)},
                        101,
                        1.01,
                        BallTrackPhase::MissingGrace,
                        ExpectedIdRelation::Same,
                        false,
                        false},
                   },
                   1100001);
    }
    {
        BallTrackState state;
        Expect(Distance(Candidate(0.10, 0.0, 1.0).world, Candidate(-0.10, 0.0, 1.0).world) <= 0.35,
               "bounce approach is within association radius");
        const Vec3 predicted_after_cross{-0.30, 0.0, 1.0};
        Expect(Distance(predicted_after_cross, Candidate(0.049, 0.0, 1.0).world) <= 0.35,
               "bounce reversal is within predicted association radius");
        CheckCases(&state,
                   {
                       {"bounce start",
                        {Candidate(0.10, 0.0, 1.0)},
                        100,
                        1.00,
                        BallTrackPhase::Active,
                        ExpectedIdRelation::Positive,
                        true,
                        false},
                       {"cross x zero with negative velocity",
                        {Candidate(-0.10, 0.0, 1.0)},
                        101,
                        1.01,
                        BallTrackPhase::Active,
                        ExpectedIdRelation::Same,
                        true,
                        false},
                       {"bounce reverses x and velocity without changing id",
                        {Candidate(0.049, 0.0, 1.0)},
                        102,
                        1.02,
                        BallTrackPhase::Active,
                        ExpectedIdRelation::Same,
                        true,
                        false},
                   },
                   1200000);
        Expect(state.velocity.x > 0.0, "bounce reverses velocity sign");
    }
    {
        BallTrackState state;
        CheckCases(&state,
                   {
                       {"shot start",
                        {Candidate(0.80, 0.0, 1.0)},
                        100,
                        1.00,
                        BallTrackPhase::Active,
                        ExpectedIdRelation::Positive,
                        true,
                        false},
                       {"shot end", {}, 125, 1.25, BallTrackPhase::Inactive, ExpectedIdRelation::Same, false, true},
                       {"next shot id increases",
                        {Candidate(0.90, 0.0, 1.0)},
                        126,
                        1.26,
                        BallTrackPhase::Active,
                        ExpectedIdRelation::Greater,
                        true,
                        false},
                   },
                   1300000);
    }
}

void TestCandidatePermutationIsDeterministic() {
    const std::vector<BallCandidate> inactive_forward{Candidate(0.80, 0.10, 1.00), Candidate(0.40, 0.20, 1.00)};
    const std::vector<BallCandidate> inactive_reverse{Candidate(0.40, 0.20, 1.00), Candidate(0.80, 0.10, 1.00)};
    BallTrackState inactive_a;
    BallTrackState inactive_b;
    const BallTrackUpdate inactive_first =
        AdvanceBallTrack(&inactive_a, inactive_forward, 100, 1.00, 0.35, 0.25, 1350000);
    const BallTrackUpdate inactive_second =
        AdvanceBallTrack(&inactive_b, inactive_reverse, 100, 1.00, 0.35, 0.25, 1350000);
    Expect(inactive_first.publish_valid && inactive_second.publish_valid, "inactive permutations both start a track");
    Expect(Distance(inactive_first.position, inactive_second.position) == 0.0,
           "inactive permutations choose the same candidate");
    Expect(Distance(inactive_first.position, Candidate(0.40, 0.20, 1.00).world) == 0.0,
           "inactive total order chooses lexicographically smallest world");

    BallTrackState active_a;
    BallTrackState active_b;
    AdvanceBallTrack(&active_a, {Candidate(1.00, 0.00, 1.00)}, 100, 1.00, 0.35, 0.25, 1360000);
    AdvanceBallTrack(&active_b, {Candidate(1.00, 0.00, 1.00)}, 100, 1.00, 0.35, 0.25, 1360000);
    const std::vector<BallCandidate> tied_forward{Candidate(1.25, 0.00, 1.00), Candidate(0.75, 0.00, 1.00)};
    const std::vector<BallCandidate> tied_reverse{Candidate(0.75, 0.00, 1.00), Candidate(1.25, 0.00, 1.00)};
    const BallTrackUpdate active_first = AdvanceBallTrack(&active_a, tied_forward, 101, 1.01, 0.35, 0.25, 1360001);
    const BallTrackUpdate active_second = AdvanceBallTrack(&active_b, tied_reverse, 101, 1.01, 0.35, 0.25, 1360001);
    Expect(active_first.publish_valid && active_second.publish_valid,
           "active equal-distance permutations both reassociate");
    Expect(Distance(active_first.position, active_second.position) == 0.0,
           "active equal-distance permutations choose the same candidate");
    Expect(Distance(active_first.position, Candidate(0.75, 0.00, 1.00).world) == 0.0,
           "active tie uses the candidate total order");
}

void TestBallCandidateRoiMatchesPythonPublisher() {
    Args args;
    const TableFrame table = IdentityTableForBallRoi();
    const double far_x = args.table_length_m - 0.40;
    const double half_width = 0.5 * args.table_width_m;
    const double epsilon = 1.0e-9;

    const std::vector<Vec3> accepted_raw{
        RawForTableWorld({far_x, half_width, args.table_height_m + epsilon}, args),
        RawForTableWorld({far_x, -half_width, args.table_height_m + 0.10}, args),
        RawForTableWorld({-0.01, 0.0, args.table_height_m + 0.10}, args),
    };
    BallSelectionDiagnostics accepted_diag;
    const std::vector<BallCandidate> accepted = BallCandidates(accepted_raw, table, args, &accepted_diag);
    Expect(accepted.size() == 3 && accepted_diag.candidate_count == 3,
           "ROI includes exact far/side boundaries and near-edge crossings");

    const std::vector<Vec3> rejected_raw{
        RawForTableWorld({far_x + epsilon, 0.0, args.table_height_m + 0.10}, args),
        RawForTableWorld({0.50, half_width + epsilon, args.table_height_m + 0.10}, args),
        RawForTableWorld({0.50, -half_width - epsilon, args.table_height_m + 0.10}, args),
        RawForTableWorld({0.50, 0.0, args.table_height_m}, args),
        RawForTableWorld({0.50, 0.0, args.table_height_m - epsilon}, args),
        Vec3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
    };
    BallSelectionDiagnostics rejected_diag;
    const std::vector<BallCandidate> rejected = BallCandidates(rejected_raw, table, args, &rejected_diag);
    Expect(rejected.empty() && rejected_diag.candidate_count == 0,
           "ROI rejects far-edge margin, table-width, height, and nonfinite points");

    BallTrackState inactive;
    const BallTrackUpdate near_edge = AdvanceBallTrack(&inactive, {accepted.back()}, 100, 1.0, 0.35, 0.25, 1);
    Expect(!near_edge.publish_valid && near_edge.phase == BallTrackPhase::Inactive,
           "near-edge crossing remains available only to an existing track");
}

void TestSourceTimeFallbacks() {
    BallTrackState state;
    AdvanceBallTrack(&state, {Candidate(0.80, 0.0, 1.0)}, 100, 1.00, 0.35, 0.25, 1400000);
    const BallTrackUpdate non_increasing =
        AdvanceBallTrack(&state, {Candidate(1.10, 0.0, 1.0)}, 103, 1.00, 0.35, 0.25, 1400001);
    Expect(non_increasing.publish_valid, "non-increasing time uses frame fallback");
    Expect(std::abs(state.velocity.x - 30.0) < 1.0e-9, "public contract uses 300 Hz fallback");

    const BallTrackUpdate non_finite =
        AdvanceBallTrack(&state, {}, 178, std::numeric_limits<double>::quiet_NaN(), 0.35, 0.25, 1400002);
    Expect(non_finite.publish_end, "non-finite time uses frame fallback");

    BallTrackState rate_state;
    AdvanceBallTrackWithFrameRate(&rate_state, {Candidate(0.80, 0.0, 1.0)}, 100, 1.00, 0.35, 0.25, 1500000, 100.0);
    const BallTrackUpdate non_300_hz = AdvanceBallTrackWithFrameRate(
        &rate_state, {}, 125, std::numeric_limits<double>::infinity(), 0.35, 0.25, 1500001, 100.0);
    Expect(non_300_hz.publish_end, "internal helper accepts selected non-300 Hz frame rate");
}

void TestAllocatorSurvivesClockRollback() {
    BallTrackState state;
    const BallTrackUpdate first = AdvanceBallTrack(&state, {Candidate(0.80, 0.0, 1.0)}, 100, 1.00, 0.35, 0.25, 2000000);
    const BallTrackUpdate ended = AdvanceBallTrack(&state, {}, 125, 1.25, 0.35, 0.25, 1999999);
    const BallTrackUpdate next = AdvanceBallTrack(&state, {Candidate(0.90, 0.0, 1.0)}, 126, 1.26, 0.35, 0.25, 1000000);
    Expect(ended.publish_end && ended.track_id == first.track_id, "end retains allocated id during clock rollback");
    Expect(next.track_id == first.track_id + 1, "allocator remains strictly monotonic during clock rollback");
}

void TestAllocatorExhaustionFailsClosed() {
    BallTrackState exhausted_state;
    exhausted_state.last_allocated_track_id = std::numeric_limits<int64_t>::max();
    const BallTrackUpdate exhausted =
        AdvanceBallTrack(&exhausted_state, {Candidate(0.80, 0.0, 1.0)}, 100, 1.00, 0.35, 0.25, 1);
    Expect(exhausted.error == BallTrackError::AllocatorExhausted,
           "INT64_MAX allocator returns explicit exhaustion error");
    Expect(!exhausted.publish_valid && !exhausted.publish_end && exhausted.track_id == 0,
           "exhausted allocator publishes no reused id");
    Expect(exhausted_state.phase == BallTrackPhase::Inactive && exhausted_state.track_id == 0 &&
               exhausted_state.last_allocated_track_id == std::numeric_limits<int64_t>::max(),
           "exhausted allocator remains fail closed");
    const BallTrackUpdate repeated = AdvanceBallTrack(&exhausted_state, {Candidate(0.90, 0.0, 1.0)}, 101, 1.01, 0.35,
                                                      0.25, std::numeric_limits<int64_t>::max());
    Expect(repeated.error == BallTrackError::AllocatorExhausted && !repeated.publish_valid && !repeated.publish_end,
           "exhausted allocator never saturates and reuses the max id");

    BallTrackState zero_time_state;
    const BallTrackUpdate zero_time =
        AdvanceBallTrack(&zero_time_state, {Candidate(0.80, 0.0, 1.0)}, 100, 1.00, 0.35, 0.25, 0);
    Expect(zero_time.error == BallTrackError::None && zero_time.publish_valid && zero_time.track_id == 1,
           "zero allocation time safely allocates id one");

    BallTrackState negative_time_state;
    const BallTrackUpdate negative_time =
        AdvanceBallTrack(&negative_time_state, {Candidate(0.80, 0.0, 1.0)}, 100, 1.00, 0.35, 0.25, -100);
    Expect(negative_time.error == BallTrackError::None && negative_time.publish_valid && negative_time.track_id == 1,
           "negative allocation time safely allocates id one");
}

void TestBaseInvalidAndMessageIds() {
    BallTrackState state;
    const BallTrackUpdate valid =
        AdvanceBallTrack(&state, {Candidate(0.80, -0.10, 1.00)}, 100, 1.00, 0.35, 0.25, 1600000);
    const BallTrackUpdate held =
        AdvanceBallTrackForBaseFrame(&state, {}, false, 1000, 10.0, 0.35, 0.25, 1600001, 300.0);
    Expect(held.phase == BallTrackPhase::Active && !held.publish_end && state.track_id == valid.track_id,
           "invalid pelvis does not advance or end ball track");
    const BallTrackUpdate ended = AdvanceBallTrackForBaseFrame(&state, {}, true, 125, 1.25, 0.35, 0.25, 1600002, 300.0);
    Expect(ended.publish_end, "valid pelvis permits source-time end");

    Args args;
    lcm_types::transformation_t pelvis{};
    lcm_types::transformation_t table{};
    lcm_types::transformation_t ball_valid{};
    lcm_types::transformation_t ball_end{};
    constexpr int64_t kPublishTimeUs = 987654321;
    FillMessage(&pelvis, kBaseSubject, {}, {}, 125, args, 300.0, kPublishTimeUs, false, true, 0);
    FillMessage(&table, "table", {}, {}, 125, args, 300.0, kPublishTimeUs, true, false, 0);
    FillMessage(&ball_valid, "ball", valid.position, {}, 100, args, 300.0, kPublishTimeUs, true, false, valid.track_id);
    FillMessage(&ball_end, "ball", ended.position, {}, 125, args, 300.0, kPublishTimeUs, false, true, ended.track_id);
    Expect(pelvis.track_id == 0 && table.track_id == 0, "pelvis and table ids are zero");
    Expect(ball_valid.track_id > 0 && ball_end.track_id == ball_valid.track_id,
           "valid and end ball messages share positive id");
    Expect(ball_valid.valid == 1 && ball_end.valid == 0, "ball end message is invalid");
    Expect(pelvis.publish_time_us == kPublishTimeUs && table.publish_time_us == kPublishTimeUs &&
               ball_valid.publish_time_us == kPublishTimeUs && ball_end.publish_time_us == kPublishTimeUs,
           "all messages from one source frame share an explicit publish time");
}

void TestBasePublicationCadenceAndTimeoutTimestamp() {
    BasePublishState state;
    const std::array<bool, 8> validity{false, false, true, true, false, false, true, false};
    const std::array<bool, 8> expected_publish{false, false, true, true, true, false, true, true};
    for (size_t i = 0; i < validity.size(); ++i) {
        const BasePublishDecision decision = AdvanceBasePublishState(&state, validity[i]);
        Expect(decision.publish == expected_publish[i],
               "base publication follows valid frames and one invalid transition");
        if (decision.publish) {
            Expect(decision.valid == validity[i], "published base validity matches the current source frame");
        }
    }

    Args args;
    constexpr int64_t kTimeoutPublishTimeUs = 123456789;
    const RuntimeFrameWaitDecision timeout =
        DecideRuntimeFrameWait(false, false, true, 321, args, 300.0, kTimeoutPublishTimeUs);
    Expect(timeout.exit_loop && timeout.exit_nonzero && timeout.publish_invalid_base,
           "stream timeout remains fail closed when publishing");
    Expect(timeout.invalid_base.publish_time_us == kTimeoutPublishTimeUs,
           "stream timeout invalid base uses the explicit timeout timestamp");
}

void TestStrictV2Args() {
    Args defaults;
    Expect(defaults.channel == "vicon_state_data_v2", "default channel is v2");
    Expect(defaults.print_hz == 1.0, "default print rate is 1 Hz");
    Expect(defaults.tracker_subject == "G1Pelvis", "default tracker is the calibrated G1Pelvis rigid body");
    Expect(UsageText("bridge").find("--pelvis-orientation-calib") != std::string::npos,
           "usage lists the pelvis orientation calibration option");

    std::vector<std::string> storage{"bridge",
                                     "--channel",
                                     "vicon_state_data_v2",
                                     "--ball-track-association-radius-m",
                                     "0.4",
                                     "--ball-track-end-timeout-s",
                                     "0.3",
                                     "--pelvis-orientation-calib",
                                     "pelvis.json"};
    std::vector<char *> argv;
    for (std::string &item : storage)
        argv.push_back(item.data());
    Args parsed;
    Expect(ParseArgs(static_cast<int>(argv.size()), argv.data(), &parsed), "strict v2 arguments parse");
    Expect(parsed.channel == kViconV2Channel && parsed.ball_track_association_radius_m == 0.4 &&
               parsed.ball_track_end_timeout_s == 0.3 && parsed.pelvis_extrinsics_path == "pelvis.json",
           "track arguments are applied");

    std::vector<std::string> numeric_storage{"bridge", "--duration",
                                             "0",      "--print-hz",
                                             "2",      "--calib-sec",
                                             "3",      "--table-length",
                                             "2.7",    "--table-width",
                                             "1.5",    "--table-height",
                                             "0.76",   "--vicon-frame-rate-hz",
                                             "300",    "--expected-base-edge-distance",
                                             "0",      "--corner-exclusion-radius-mm",
                                             "0"};
    std::vector<char *> numeric_argv;
    for (std::string &item : numeric_storage)
        numeric_argv.push_back(item.data());
    Args numeric;
    Expect(ParseArgs(static_cast<int>(numeric_argv.size()), numeric_argv.data(), &numeric),
           "strict numeric arguments accept their valid zero boundaries");

    std::vector<std::string> split_subject_storage{"bridge", "--tracker-name", "G1Pelvis", "--base-subject",
                                                   "G2Pelvis"};
    std::vector<char *> split_subject_argv;
    for (std::string &item : split_subject_storage) {
        split_subject_argv.push_back(item.data());
    }
    Args split_subject;
    Expect(ParseArgs(static_cast<int>(split_subject_argv.size()), split_subject_argv.data(), &split_subject),
           "tracker subject can differ from published base subject");
    Expect(split_subject.tracker_subject == "G1Pelvis" && split_subject.base_subject == "G2Pelvis",
           "tracker subject and published base subject are separated");

    std::vector<std::string> wrong_tracker_storage{"bridge", "--tracker-name", "G2Pelvis"};
    std::vector<char *> wrong_tracker_argv;
    for (std::string &item : wrong_tracker_storage) {
        wrong_tracker_argv.push_back(item.data());
    }
    Args wrong_tracker;
    Expect(!ParseArgs(static_cast<int>(wrong_tracker_argv.size()), wrong_tracker_argv.data(), &wrong_tracker),
           "tracker name is bound to the calibrated G1Pelvis rigid body");

    std::vector<std::string> conflicting_table_storage{"bridge", "--table-calib", "loaded.json", "--save-table-calib",
                                                       "saved.json"};
    std::vector<char *> conflicting_table_argv;
    for (std::string &item : conflicting_table_storage) {
        conflicting_table_argv.push_back(item.data());
    }
    Args conflicting_table;
    Expect(
        !ParseArgs(static_cast<int>(conflicting_table_argv.size()), conflicting_table_argv.data(), &conflicting_table),
        "loading and saving table calibration are mutually exclusive");

    std::vector<std::string> conflicting_role_storage{"bridge", "--pelvis-orientation-calib",
                                                      "/tmp/robotbridge4-calib.json", "--save-table-calib",
                                                      "/tmp/./robotbridge4-calib.json"};
    std::vector<char *> conflicting_role_argv;
    for (std::string &item : conflicting_role_storage) {
        conflicting_role_argv.push_back(item.data());
    }
    Args conflicting_role;
    Expect(!ParseArgs(static_cast<int>(conflicting_role_argv.size()), conflicting_role_argv.data(), &conflicting_role),
           "calibration roles cannot resolve to the same file");

    std::vector<std::string> wrong_storage{"bridge", "--channel", "vicon_state_data"};
    std::vector<char *> wrong_argv;
    for (std::string &item : wrong_storage)
        wrong_argv.push_back(item.data());
    Args wrong;
    Expect(!ParseArgs(static_cast<int>(wrong_argv.size()), wrong_argv.data(), &wrong), "v1 channel is rejected");

    std::vector<std::string> zero_storage{"bridge", "--ball-track-end-timeout-s", "0"};
    std::vector<char *> zero_argv;
    for (std::string &item : zero_storage)
        zero_argv.push_back(item.data());
    Args zero;
    Expect(!ParseArgs(static_cast<int>(zero_argv.size()), zero_argv.data(), &zero),
           "non-positive track timeout is rejected");

    std::vector<std::string> nan_storage{"bridge", "--ball-track-association-radius-m", "nan"};
    std::vector<char *> nan_argv;
    for (std::string &item : nan_storage)
        nan_argv.push_back(item.data());
    Args nan;
    Expect(!ParseArgs(static_cast<int>(nan_argv.size()), nan_argv.data(), &nan),
           "non-finite association radius is rejected");

    std::vector<std::string> infinity_storage{"bridge", "--ball-track-end-timeout-s", "inf"};
    std::vector<char *> infinity_argv;
    for (std::string &item : infinity_storage) {
        infinity_argv.push_back(item.data());
    }
    Args infinity;
    Expect(!ParseArgs(static_cast<int>(infinity_argv.size()), infinity_argv.data(), &infinity),
           "non-finite track timeout is rejected");

    struct InvalidArgsCase {
        const char *name;
        std::vector<std::string> args;
    };
    const std::vector<InvalidArgsCase> invalid_cases{
        {"missing channel value", {"bridge", "--channel"}},
        {"missing association value", {"bridge", "--ball-track-association-radius-m"}},
        {"missing timeout value", {"bridge", "--ball-track-end-timeout-s"}},
        {"non-numeric association value", {"bridge", "--ball-track-association-radius-m", "abc"}},
        {"trailing association characters", {"bridge", "--ball-track-association-radius-m", "0.35junk"}},
        {"non-numeric timeout value", {"bridge", "--ball-track-end-timeout-s", "abc"}},
        {"trailing timeout characters", {"bridge", "--ball-track-end-timeout-s", "0.25junk"}},
        {"missing pelvis calibration value", {"bridge", "--pelvis-orientation-calib"}},
        {"missing host value", {"bridge", "--host"}},
        {"option cannot become host value", {"bridge", "--host", "--publish"}},
        {"missing table calibration value", {"bridge", "--table-calib"}},
        {"missing save table calibration value", {"bridge", "--save-table-calib"}},
        {"missing lcm url value", {"bridge", "--lcm-url"}},
        {"missing raw ignore sphere value", {"bridge", "--ignore-raw-sphere-m"}},
    };
    for (const InvalidArgsCase &test : invalid_cases) {
        std::vector<std::string> args_storage = test.args;
        std::vector<char *> args_argv;
        for (std::string &item : args_storage)
            args_argv.push_back(item.data());
        Args invalid;
        Expect(!ParseArgs(static_cast<int>(args_argv.size()), args_argv.data(), &invalid),
               std::string(test.name) + " returns false without escaping parser");
    }

    struct NumericArgCase {
        const char *option;
        bool zero_allowed;
    };
    const std::vector<NumericArgCase> numeric_cases{
        {"--duration", true},
        {"--print-hz", false},
        {"--calib-sec", false},
        {"--table-length", false},
        {"--table-width", false},
        {"--table-height", false},
        {"--vicon-frame-rate-hz", false},
        {"--expected-base-edge-distance", true},
        {"--corner-exclusion-radius-mm", true},
    };
    for (const NumericArgCase &numeric_case : numeric_cases) {
        const std::vector<std::string> invalid_values{"abc", "nan", "inf", "1junk", "-1"};
        for (const std::string &value : invalid_values) {
            std::vector<std::string> args_storage{"bridge", numeric_case.option, value};
            std::vector<char *> args_argv;
            for (std::string &item : args_storage)
                args_argv.push_back(item.data());
            Args invalid;
            Expect(!ParseArgs(static_cast<int>(args_argv.size()), args_argv.data(), &invalid),
                   std::string(numeric_case.option) + " rejects " + value);
        }
        if (!numeric_case.zero_allowed) {
            std::vector<std::string> args_storage{"bridge", numeric_case.option, "0"};
            std::vector<char *> args_argv;
            for (std::string &item : args_storage)
                args_argv.push_back(item.data());
            Args invalid;
            Expect(!ParseArgs(static_cast<int>(args_argv.size()), args_argv.data(), &invalid),
                   std::string(numeric_case.option) + " rejects zero");
        }

        std::vector<std::string> missing_storage{"bridge", numeric_case.option};
        std::vector<char *> missing_argv;
        for (std::string &item : missing_storage)
            missing_argv.push_back(item.data());
        Args missing;
        Expect(!ParseArgs(static_cast<int>(missing_argv.size()), missing_argv.data(), &missing),
               std::string(numeric_case.option) + " rejects a missing value without terminating");
    }

    Args table_only;
    table_only.save_table_calib_path = "candidate.json";
    Expect(IsTableOnlyCalibration(table_only), "save-only no-publish invocation is table-only calibration");
    table_only.pelvis_extrinsics_path = "pelvis.json";
    Expect(!IsTableOnlyCalibration(table_only), "loading pelvis calibration selects runtime like Python");
    table_only.pelvis_extrinsics_path.clear();
    table_only.publish = true;
    Expect(!IsTableOnlyCalibration(table_only), "publishing is never table-only calibration");
}

void TestTableRectangleScoreMatchesPythonDefinition() {
    TableDimensions dimensions;
    const double length_mm = dimensions.length_m * 1000.0;
    const double width_mm = dimensions.width_m * 1000.0;
    std::array<Vec3, 4> exact{
        Vec3{0.0, 0.0, 0.0},
        Vec3{length_mm, 0.0, 0.0},
        Vec3{0.0, width_mm, 0.0},
        Vec3{length_mm, width_mm, 0.0},
    };
    Expect(std::abs(TableRectangleScore(exact, dimensions)) < 1.0e-12,
           "exact planar table rectangle has zero Python-compatible score");

    exact[3].z = 30.0;
    const double perturbed_score = TableRectangleScore(exact, dimensions);
    Expect(std::isfinite(perturbed_score) && perturbed_score >= 0.10,
           "height spread contributes to the Python-compatible score");
}

void TestLoadsPythonTableCalibration() {
    const std::string calibration_path = "/tmp/robotbridge4_test_table.json";
    {
        std::ofstream output(calibration_path, std::ios::trunc);
        Expect(static_cast<bool>(output), "opens temporary table calibration");
        output << "{\n"
               << "  \"format\": \"" << kTableCalibrationFormat << "\",\n"
               << "  \"units\": \"metres\",\n"
               << "  \"table_length_m\": 2.730738,\n"
               << "  \"table_width_m\": 1.512451,\n"
               << "  \"table_height_m\": 0.76,\n"
               << "  \"center_raw_m\": [1.365369, 0.0, 0.76],\n"
               << "  \"x_axis_raw\": [1.0, 0.0, 0.0],\n"
               << "  \"y_axis_raw\": [0.0, 1.0, 0.0],\n"
               << "  \"z_axis_raw\": [0.0, 0.0, 1.0],\n"
               << "  \"corners_raw_m\": [\n"
               << "    [0.0, -0.7562255, 0.76],\n"
               << "    [0.0, 0.7562255, 0.76],\n"
               << "    [2.730738, -0.7562255, 0.76],\n"
               << "    [2.730738, 0.7562255, 0.76]\n"
               << "  ],\n"
               << "  \"rectangle_score\": 0.0\n"
               << "}\n";
        Expect(static_cast<bool>(output), "writes temporary table calibration");
    }
    TableDimensions dimensions;
    TableFrame table;
    std::string error;
    Expect(LoadTableCalibration(calibration_path, &dimensions, &table, &error),
           std::string("loads Python-format table calibration: ") + error);
    Expect(dimensions.height_m == 0.76, "Python table_height_m is used as table height");
    Expect(table.valid, "loaded table is valid");
}

void TestWritesPythonCompatibleTableCalibration() {
    TableDimensions dimensions;
    TableFrame table;
    table.center_raw_mm = {1000.0, 2000.0, 3000.0};
    table.x_axis_raw = {1.0, 0.0, 0.0};
    table.y_axis_raw = {0.0, 1.0, 0.0};
    table.z_axis_raw = {0.0, 0.0, 1.0};
    table.corners_raw_mm = {
        Vec3{0.0, 0.0, 0.0},
        Vec3{dimensions.length_m * 1000.0, 0.0, 0.0},
        Vec3{0.0, dimensions.width_m * 1000.0, 0.0},
        Vec3{dimensions.length_m * 1000.0, dimensions.width_m * 1000.0, 0.0},
    };
    table.rectangle_score = 0.123456789;
    table.corner_max_error_m = 0.01;
    table.corner_rms_error_m = 0.005;

    std::ostringstream output;
    Expect(WriteCalibrationJson(output, dimensions, table), "writes table calibration JSON");
    const std::string json = output.str();
    Expect(json.find("\"format\": \"robotbridge2_chingmu_table_frame_v1\"") != std::string::npos,
           "written table calibration declares the Python schema format");
    Expect(json.find("\"units\": \"metres\"") != std::string::npos, "written table calibration declares metre units");
    Expect(json.find("\"table_height_m\": 0.7600000000") != std::string::npos,
           "written table calibration includes Python table_height_m");
    Expect(json.find("\"rectangle_score\": 0.1234567890") != std::string::npos,
           "written table calibration includes the inferred rectangle score");
    Expect(json.find("\"height_m_estimate\": 0.7600000000") != std::string::npos,
           "written table calibration preserves the legacy height field");
}

void TestLoadsAndAppliesPythonPelvisCalibration() {
    const std::string calibration_path = "/tmp/robotbridge4_test_pelvis.json";
    {
        std::ofstream output(calibration_path, std::ios::trunc);
        Expect(static_cast<bool>(output), "opens temporary pelvis calibration");
        output << "{\n"
               << "  \"angular_rms_deg\": 0.1196313639510382,\n"
               << "  \"base_subject\": \"G2Pelvis\",\n"
               << "  \"format\": \"robotbridge2_chingmu_pelvis_orientation_v1\",\n"
               << "  \"position_rms_m\": 0.00016156891287081543,\n"
               << "  \"quaternion_convention\": \"xyzw\",\n"
               << "  \"quaternion_rigid_from_pelvis_xyzw\": "
                  "[-0.0074216477824771325, -0.008735911075409388, "
                  "0.17337147908657494, 0.9847897913977423],\n"
               << "  \"rotation_convention\": \"" << kPelvisRotationConvention << "\",\n"
               << "  \"sample_count\": 1498,\n"
               << "  \"source_duration_s\": 4.99,\n"
               << "  \"translation_convention\": \"" << kPelvisTranslationConvention << "\",\n"
               << "  \"translation_rigid_origin_to_pelvis_origin_pelvis_m\": "
                  "[0.0, 0.0, 0.07605]\n"
               << "}\n";
        Expect(static_cast<bool>(output), "writes temporary pelvis calibration");
    }
    PelvisExtrinsics extrinsics;
    std::string error;
    Expect(LoadPelvisExtrinsics(calibration_path, kBaseSubject, &extrinsics, &error),
           std::string("loads Python-generated pelvis calibration: ") + error);
    Expect(extrinsics.valid, "loaded pelvis extrinsics are valid");
    Expect(std::abs(extrinsics.translation_pelvis_m.z - 0.07605) < 1.0e-12,
           "pelvis origin translation is loaded in metres");

    Args args;
    TableFrame table = IdentityTableForBallRoi();
    table.center_raw_mm = {0.0, 0.0, 0.0};
    SegmentPoseRaw raw;
    raw.translation_mm = {100.0, 200.0, 300.0};
    raw.quat_xyzw = {0.0, 0.0, 0.0, 1.0};
    raw.valid = true;
    raw.occluded = false;

    const BasePoseWorld transformed = TransformRootPose(raw, table, args, extrinsics);
    Expect(transformed.valid, "extrinsic pelvis transform is valid");
    double expected_rotation[3][3];
    QuatToMatrix(extrinsics.rotation_rigid_from_pelvis, expected_rotation);
    const Vec3 expected_offset{
        expected_rotation[0][0] * extrinsics.translation_pelvis_m.x +
            expected_rotation[0][1] * extrinsics.translation_pelvis_m.y +
            expected_rotation[0][2] * extrinsics.translation_pelvis_m.z,
        expected_rotation[1][0] * extrinsics.translation_pelvis_m.x +
            expected_rotation[1][1] * extrinsics.translation_pelvis_m.y +
            expected_rotation[1][2] * extrinsics.translation_pelvis_m.z,
        expected_rotation[2][0] * extrinsics.translation_pelvis_m.x +
            expected_rotation[2][1] * extrinsics.translation_pelvis_m.y +
            expected_rotation[2][2] * extrinsics.translation_pelvis_m.z,
    };
    const Vec3 rigid_position = RawToTableWorld(raw.translation_mm, table, args);
    Expect(Distance(transformed.position_m, rigid_position + expected_offset) < 1.0e-12,
           "pelvis translation is rotated by the published pelvis orientation");
    Expect(std::abs(transformed.orientation_xyzw.x - extrinsics.rotation_rigid_from_pelvis.x) < 1.0e-12 &&
               std::abs(transformed.orientation_xyzw.y - extrinsics.rotation_rigid_from_pelvis.y) < 1.0e-12 &&
               std::abs(transformed.orientation_xyzw.z - extrinsics.rotation_rigid_from_pelvis.z) < 1.0e-12 &&
               std::abs(transformed.orientation_xyzw.w - extrinsics.rotation_rigid_from_pelvis.w) < 1.0e-12,
           "published pelvis orientation applies the calibrated correction");
}

void TestRejectsPelvisCalibrationBoundToAnotherSubject() {
    const std::string source_path = "/tmp/robotbridge4_test_pelvis.json";

    PelvisExtrinsics extrinsics;
    std::string error;
    Expect(!LoadPelvisExtrinsics(source_path, "OtherPelvis", &extrinsics, &error),
           "pelvis calibration cannot be applied to another published subject");
    Expect(!extrinsics.valid, "rejected pelvis calibration stays invalid");
}

std::vector<BallTrackUpdate> ContractFixture() {
    BallTrackState state;
    std::vector<BallTrackUpdate> updates;
    updates.push_back(AdvanceBallTrack(&state, {Candidate(0.80, -0.10, 1.00)}, 100, 1.00, 0.35, 0.25, 1000000));
    updates.push_back(AdvanceBallTrack(&state, {Candidate(0.50, -0.10, 1.00)}, 101, 1.01, 0.35, 0.25, 1000001));
    updates.push_back(AdvanceBallTrack(&state, {}, 102, 1.02, 0.35, 0.25, 1000002));
    updates.push_back(AdvanceBallTrack(&state, {Candidate(-0.02, -0.10, 1.00)}, 103, 1.03, 0.35, 0.25, 1000003));
    updates.push_back(AdvanceBallTrack(&state, {}, 128, 1.28, 0.35, 0.25, 1000004));
    updates.push_back(AdvanceBallTrack(&state, {Candidate(0.90, 0.20, 1.10)}, 129, 1.29, 0.35, 0.25, 1000100));
    return updates;
}

void EmitTrackFixture() {
    const std::vector<BallTrackUpdate> updates = ContractFixture();
    std::cout << "[";
    for (size_t index = 0; index < updates.size(); ++index) {
        const BallTrackUpdate &update = updates[index];
        if (index != 0)
            std::cout << ",";
        std::cout << "{\"phase\":\"" << BallTrackPhaseName(update.phase) << "\",\"track_id\":" << update.track_id
                  << ",\"publish_valid\":" << (update.publish_valid ? "true" : "false")
                  << ",\"publish_end\":" << (update.publish_end ? "true" : "false") << ",\"position_world\":["
                  << update.position.x << "," << update.position.y << "," << update.position.z << "]}";
    }
    std::cout << "]\n";
}

} // namespace

int main(int argc, char **argv) {
    if (argc == 2 && std::string(argv[1]) == "--emit-track-fixture") {
        EmitTrackFixture();
        return 0;
    }
    TestTrackIdentityAndGrace();
    TestTableDrivenTrackContract();
    TestCandidatePermutationIsDeterministic();
    TestBallCandidateRoiMatchesPythonPublisher();
    TestSourceTimeFallbacks();
    TestAllocatorSurvivesClockRollback();
    TestAllocatorExhaustionFailsClosed();
    TestBaseInvalidAndMessageIds();
    TestBasePublicationCadenceAndTimeoutTimestamp();
    TestStrictV2Args();
    TestTableRectangleScoreMatchesPythonDefinition();
    TestLoadsPythonTableCalibration();
    TestWritesPythonCompatibleTableCalibration();
    TestLoadsAndAppliesPythonPelvisCalibration();
    TestRejectsPelvisCalibrationBoundToAnotherSubject();
    std::cout << "PASS: Vicon v2 ball track contract\n";
    return 0;
}
